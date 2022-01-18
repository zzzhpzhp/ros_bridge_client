#include "ros_bridge_client.h"

namespace ros_bridge_client
{
    ROSBridgeClient::ROSBridgeClient()
    {
        context_ = zmq_ctx_new();
        publisher_ = zmq_socket(context_, ZMQ_PUB);
        subscriber_ = zmq_socket(context_, ZMQ_SUB);
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
            auto recv_body = _recv_data(subscriber_, 0);
            if (!recv_body)
            {
                continue;
            }

            _msg_handle(recv_body);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
        zmq_send(publisher_, namespace_.data(), namespace_.size(), ZMQ_DONTWAIT | ZMQ_SNDMORE);
        zmq_send(publisher_, body.data(), body.size(), ZMQ_DONTWAIT);
        publisher_mtx_.unlock();
    }

    void ROSBridgeClient::_msg_handle(char *msg)
    {
        sub_root_.clear();
        if (!reader_.parse(msg, sub_root_))
        {
            std::cerr << "Environment config file parse failed." << std::endl;
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

}
