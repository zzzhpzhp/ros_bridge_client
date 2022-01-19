#include "ros_bridge_client.h"

namespace ros_bridge_client
{
    ROSBridgeClient::ROSBridgeClient()
    {
        context_ = zmq_ctx_new();
        publisher_ = zmq_socket(context_, ZMQ_PUB);
        subscriber_ = zmq_socket(context_, ZMQ_SUB);
    }

    bool ROSBridgeClient::initialize()
    {
        Json::Reader reader;
        Json::Value param_root;
        //从文件中读取，保证当前文件有demo.json文件
        std::ifstream param_stream("../param.json", std::ios::in);

        if (param_stream.is_open())
        {
            if (reader.parse(param_stream, param_root))
            {
                if (!param_root["namespace"].empty())
                {
                    namespace_ = param_root["namespace"].asString();
                    std::cout << "Get namespace from parameters file: " << namespace_ << std::endl;
                }

                if (!param_root["pub_address"].empty())
                {
                    pub_address_ = param_root["pub_address"].asString();
                    std::cout << "Get publish address from parameters file: " << pub_address_ << std::endl;
                }

                if (!param_root["sub_address"].empty())
                {
                    sub_address_ = param_root["sub_address"].asString();
                    std::cout << "Get subscribe address from parameters file: " << sub_address_ << std::endl;
                }

                if (!param_root["frequency"].empty())
                {
                    frequency_ = param_root["frequency"].asFloat();
                    interval_ = 1.0f / frequency_;
                    std::cout << "Get frequency from parameters file: " << frequency_ << std::endl;
                }
            }
        }
        else
        {
            std::cout << "Error opening file\n";
        }



        res_ = zmq_bind(publisher_, pub_address_.c_str());
        if (res_ != 0)
        {
            std::cerr << "Failed bind to " << pub_address_ << std::endl;
            return false;
        }
        std::cout << "Succeed bind to publish address " << pub_address_ << std::endl;

        res_ = zmq_connect(subscriber_, sub_address_.c_str());
        res_ = zmq_setsockopt(subscriber_, ZMQ_SUBSCRIBE, namespace_.data(), namespace_.size());
        if (res_ != 0)
        {
            std::cerr << "Failed connect to " << sub_address_ << std::endl;
            return false;
        }
        std::cout << "Succeed subscribe to address " << sub_address_ << std::endl;

        return true;
    }

    void ROSBridgeClient::publishPoint(std::string topic_name, std::string frame_id, float x, float y,
                                       float z)
    {
        Json::Value msg;
        msg["topic_name"] = topic_name;
        msg["type"] = "geometry_msgs::PointStamped";
        msg["header"]["frame_id"] = frame_id;

        msg["point"]["x"] = x;
        msg["point"]["y"] = y;
        msg["point"]["z"] = z;

        _publish_msg(msg);
    }

    void ROSBridgeClient::start()
    {
        is_running_.store(true);
        recv_thread_ = std::thread(boost::bind(&ROSBridgeClient::_run, this));
        recv_thread_.detach();
    }

    void ROSBridgeClient::stop()
    {
        std::cout << "Stop ros bridge server..." << std::endl;
        is_running_.store(false);
        zmq_close(subscriber_);
        zmq_close(publisher_);
        zmq_ctx_destroy(context_);
        while(!already_stop_.load());
        std::cout << "ROS bridge server stoped." << std::endl;
    }

    void ROSBridgeClient::setTopicCallback(std::string topic_name,
                                           ROSBridgeClient::TopicCallbackType callback)
    {
//        printf("Setting callback for topic %s", topic_name.c_str());
        std::cout << "Setting callback for topic " << topic_name << std::endl;
        recv_topic_cb_[topic_name] = callback;
    }

    void ROSBridgeClient::_run()
    {
        already_stop_.store(false);
        while (is_running_.load())
        {
            auto recv_header = _recv_data(subscriber_, 0);
            if (!recv_header)
            {
                continue;
            }
            free(recv_header);
            auto recv_body = _recv_data(subscriber_, 0);
            if (!recv_body)
            {
                continue;
            }

            _msg_handle(recv_body);
            free(recv_body);

            sleep(interval_);
        }
        already_stop_.store(true);
    }

    char *ROSBridgeClient::_recv_data(void *socket, int opt)
    {
        // 创建zmq_msg_t对象接收数据
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        int size = zmq_msg_recv(&msg, socket, opt);
        if(size == -1)
        {
            return nullptr;
        }

        // 将zmq_msg_t对象中的数据保存到字符串中
        char *data = (char*)malloc(size + 1);
        memcpy(data, zmq_msg_data(&msg), size);

        zmq_msg_close(&msg);
        data[size] = 0;

        return data;
    }

    void ROSBridgeClient::_publish_msg(const Json::Value &data)
    {
        publisher_mtx_.lock();
        auto body = fw_.write(data);
        auto send_size = zmq_send(publisher_, namespace_.data(), namespace_.size(), ZMQ_DONTWAIT | ZMQ_SNDMORE);
        send_size = zmq_send(publisher_, body.data(), body.size(), ZMQ_DONTWAIT);
        publisher_mtx_.unlock();
    }

    void ROSBridgeClient::_msg_handle(char *msg)
    {
        sub_root_.clear();
        if (!reader_.parse(msg, sub_root_))
        {
            std::cerr << "Parse message failed." << std::endl;
        }
        auto topic_name = sub_root_["topic_name"].asString();

        if (recv_topic_cb_.count(topic_name) > 0)
        {
            recv_topic_cb_[topic_name](sub_root_);
        }
    }

    void ROSBridgeClient::publishTf(std::string topic_name, std::string frame_id, std::string child_frame_id, float x,
                                    float y, float theta)
    {
        Json::Value msg;
        msg["topic_name"] = topic_name;
        msg["type"] = "tf::tfMessage";
        msg["child_frame_id"] = child_frame_id;
        msg["header"]["frame_id"] = frame_id;

        Json::Value trans;
        trans["rotation"]["theta"] = theta;
        trans["translation"]["x"] = x;
        trans["translation"]["y"] = y;
        trans["translation"]["z"] = 0.0f;

        msg["transforms"].append(trans);

        _publish_msg(msg);
    }

    void ROSBridgeClient::publishPose(std::string topic_name, std::string frame_id, float x, float y, float theta)
    {
        Json::Value msg;
        msg["topic_name"] = topic_name;
        msg["type"] = "geometry_msgs::PoseStamped";
        msg["header"]["frame_id"] = frame_id;

        msg["pose"]["position"]["x"] = x;
        msg["pose"]["position"]["y"] = y;
        msg["pose"]["position"]["z"] = 0.0f;
        msg["pose"]["orientation"]["theta"] = theta;

        _publish_msg(msg);
    }

    void ROSBridgeClient::publishOccupancyGrid(std::string topic_name, std::string frame_id, int height, int width,
                                               float resolution, float origin_x, float origin_y, float origin_theta,
                                               const std::vector<uint8_t> &data)
    {
        Json::Value msg;
        msg["topic_name"] = topic_name;
        msg["type"] = "nav_msgs::OccupancyGrid";
        msg["header"]["frame_id"] = frame_id;

        msg["height"] = height;
        msg["width"] = width;
        msg["resolution"] = resolution;
        msg["position"]["x"] = origin_x;
        msg["position"]["y"] = origin_y;
        msg["position"]["z"] = 0.0f;
        msg["orientation"]["theta"] = origin_theta;

        std::string str;
        str.assign(data.begin(), data.end());
        msg["data"] = str;

        _publish_msg(msg);
    }

    void
    ROSBridgeClient::publishScan(std::string topic_name, std::string frame_id, float scan_time, float time_increment,
                                 float angle_increment, float angle_max, float angle_min, float range_max,
                                 float range_min, std::vector<float> ranges, std::vector<float> intensities)
    {
        Json::Value msg;
        msg["topic_name"] = topic_name;
        msg["type"] = "sensor_msgs::LaserScan";
        msg["header"]["frame_id"] = frame_id;

        msg["scan_time"] = scan_time;
        msg["time_increment"] = time_increment;
        msg["angle_increment"] = angle_increment;
        msg["angle_max"] = angle_max;
        msg["angle_min"] = angle_min;
        msg["range_max"] = range_max;
        msg["range_min"] = range_min;

        auto &rs = msg["ranges"];
        for (const auto &r : ranges)
        {
            rs.append(r);
        }

        auto &is = msg["intensities"];
        for (const auto &i : intensities)
        {
            is.append(i);
        }

        _publish_msg(msg);
    }

}
