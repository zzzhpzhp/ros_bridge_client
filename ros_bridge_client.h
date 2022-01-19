#pragma once

// c++
// http://www.cplusplus.com/reference/
//#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <atomic>
#include <mutex>
#include <thread>
#include <functional>
#include <unordered_map>
//#include <codecvt>
//#include <tuple>
//#include <complex>
//#include <exception>
//#include <initializer_list>
//#include <iterator>
//#include <limits>
//#include <locale>
//#include <new>
//#include <numeric>
//#include <random>
//#include <ratio>
//#include <regex>
//#include <stdexcept>
//#include <system_error>
//#include <typeindex>
//#include <typeinfo>
//#include <type_traits>
//#include <utility>
//#include <valarray>
//#include <condition_variable>
//#include <future>
#include <iostream>

#include "boost/bind.hpp"

#include "string.h"
#include <zmq.h>
#include <jsoncpp/json/json.h>

namespace ros_bridge_client
{
    enum MarkerAction {ADD = 0, MODIFY, DELETE, DELETEALL};
    enum MarkerType {ARROW = 0, CUBE, SPHERE, CYLINDER, LINE_STRIP, LINE_LIST, CUBE_LIST, SPHERE_LIST, POINTS, TEXT_VIEW_FACING, MESH_RESOURCE, TRIANGLE_LIST};

    class ROSBridgeClient
    {
    private:

        using TopicCallbackType = std::function<void(Json::Value&)>;

    public:

        ROSBridgeClient();

        bool initialize();

        void start();

        void stop();

        void publishPoint(std::string topic_name, std::string frame_id, float x, float y, float z = 0.0f);

        void publishTf(std::string topic_name, std::string frame_id, std::string child_frame_id,
                       float x, float y, float theta);

        void publishScan(std::string topic_name, std::string frame_id)
        {

        }

        void publishPose(std::string topic_name, std::string frame_id, float x, float y, float theta);

        template<typename T>
        void publishPath(std::string topic_name, std::string frame_id, const T &points)
        {
            Json::Value msg;
            msg["topic_name"] = topic_name;
            msg["type"] = "nav_msgs::Path";
            msg["header"]["frame_id"] = frame_id;

            Json::Value point;
            for (const auto &p : points)
            {
                point["x"] = p.x;
                point["y"] = p.y;
                msg["poses"].append(point);
            }

            _publish_msg(msg);
        }

        template<typename T>
        void publishPoses(std::string topic_name, std::string frame_id, const T &poses)
        {
            Json::Value msg;
            msg["topic_name"] = topic_name;
            msg["type"] = "geometry_msgs::PoseArray";
            msg["header"]["frame_id"] = frame_id;

            Json::Value pose;
            for (const auto &p : poses)
            {
                pose["x"] = p.x;
                pose["y"] = p.y;
                pose["z"] = 0.0f;
                pose["theta"] = p.theta;

                msg["poses"].append(pose);
            }

            _publish_msg(msg);
        }

        template<typename T>
        void publishPoints(std::string topic_name, std::string frame_id, const T &points, float size_x, float size_y, float r, float g, float b, float a = 1.0)
        {
            Json::Value msg;
            msg["topic_name"] = topic_name;
            msg["type"] = "visualization_msgs::Marker";
            msg["header"]["frame_id"] = frame_id;
            msg["ns"] = "points";
            msg["id"] = 0;
            msg["action"] = ADD;
            msg["marker_type"] = SPHERE_LIST;
            msg["color"]["r"] = r;
            msg["color"]["g"] = g;
            msg["color"]["b"] = b;
            msg["color"]["a"] = a;

            // 偏移值
            msg["pose"]["position"]["x"] = 0.0f;
            msg["pose"]["position"]["y"] = 0.0f;
            msg["pose"]["position"]["z"] = 0.0f;
            msg["pose"]["orientation"]["x"] = 0.0f;
            msg["pose"]["orientation"]["y"] = 0.0f;
            msg["pose"]["orientation"]["z"] = 0.0f;
            msg["pose"]["orientation"]["w"] = 1.0f;

            // 尺寸不得小于等于0
            size_x = std::max(0.01f, size_x);
            size_y = std::max(0.01f, size_y);
            msg["scale"]["x"] = size_x;
            msg["scale"]["y"] = size_y;
            msg["scale"]["z"] = 1.0f;

            Json::Value point;
            for (const auto &p : points)
            {
                point["x"] = p.x;
                point["y"] = p.y;
                point["z"] = 0.0f;

                msg["points"].append(point);
            }

            _publish_msg(msg);
        }

        template<typename T>
        void publishLineStrip(std::string topic_name, std::string frame_id, const T &points, float size_x, float r, float g, float b, float a = 1.0)
        {
            Json::Value msg;
            msg["topic_name"] = topic_name;
            msg["type"] = "visualization_msgs::Marker";
            msg["header"]["frame_id"] = frame_id;
            msg["ns"] = "line_strip";
            msg["id"] = 0;
            msg["action"] = ADD;
            msg["marker_type"] = LINE_STRIP;
            msg["color"]["r"] = r;
            msg["color"]["g"] = g;
            msg["color"]["b"] = b;
            msg["color"]["a"] = a;

            // 偏移值
            msg["pose"]["position"]["x"] = 0.0f;
            msg["pose"]["position"]["y"] = 0.0f;
            msg["pose"]["position"]["z"] = 0.0f;
            msg["pose"]["orientation"]["x"] = 0.0f;
            msg["pose"]["orientation"]["y"] = 0.0f;
            msg["pose"]["orientation"]["z"] = 0.0f;
            msg["pose"]["orientation"]["w"] = 1.0f;

            // 尺寸不得小于等于0
            size_x = std::max(0.01f, size_x);
            msg["scale"]["x"] = size_x;

            Json::Value point;
            for (const auto &p : points)
            {
                point["x"] = p.x;
                point["y"] = p.y;
                point["z"] = 0.0f;

                msg["points"].append(point);
            }

            _publish_msg(msg);
        }

        template<typename T>
        void publishLineList(std::string topic_name, std::string frame_id, const T &points, float size_x, float r, float g, float b, float a = 1.0)
        {
            if (points.size() %2 != 0)
            {
                std::cerr << "Line list points size should be odd." << std::endl;
            }
            Json::Value msg;
            msg["topic_name"] = topic_name;
            msg["type"] = "visualization_msgs::Marker";
            msg["header"]["frame_id"] = frame_id;
            msg["ns"] = "line_list";
            msg["id"] = 0;
            msg["action"] = ADD;
            msg["marker_type"] = LINE_LIST;
            msg["color"]["r"] = r;
            msg["color"]["g"] = g;
            msg["color"]["b"] = b;
            msg["color"]["a"] = a;

            // 偏移值
            msg["pose"]["position"]["x"] = 0.0f;
            msg["pose"]["position"]["y"] = 0.0f;
            msg["pose"]["position"]["z"] = 0.0f;
            msg["pose"]["orientation"]["x"] = 0.0f;
            msg["pose"]["orientation"]["y"] = 0.0f;
            msg["pose"]["orientation"]["z"] = 0.0f;
            msg["pose"]["orientation"]["w"] = 1.0f;

            // 尺寸不得小于等于0
            size_x = std::max(0.01f, size_x);
            msg["scale"]["x"] = size_x;

            Json::Value point;
            for (const auto &p : points)
            {
                point["x"] = p.x;
                point["y"] = p.y;
                point["z"] = 0.0f;

                msg["points"].append(point);
            }

            _publish_msg(msg);
        }


        template<typename T>
        void publishPolygon(std::string topic_name, std::string frame_id, const T &points)
        {
            Json::Value msg;
            msg["topic_name"] = topic_name;
            msg["type"] = "geometry_msgs::PolygonStamped";
            msg["header"]["frame_id"] = frame_id;

            Json::Value point;
            for (const auto &p : points)
            {
                point["x"] = p.x;
                point["y"] = p.y;
                msg["points"].append(point);
            }

            _publish_msg(msg);
        }

        void setTopicCallback(std::string topic_name, TopicCallbackType callback);

    private:

        Json::Value sub_root_;
        Json::FastWriter fw_;
        Json::Reader reader_;
        std::string namespace_{"robot"};
        std::string sub_address_{"tcp://localhost:5557"};
        std::string pub_address_{"tcp://*:5556"};
        void *context_, *publisher_, *subscriber_;
        int res_;
        std::mutex publisher_mtx_;

        std::atomic_bool is_running_{false}, already_stop_{false};
        std::thread recv_thread_;

        std::unordered_map<std::string, TopicCallbackType> recv_topic_cb_;

    private:

        void _run();

        char *_recv_data(void *socket, int opt = 0);

        void _publish_msg(const Json::Value &data);

        void _msg_handle(char *msg);
    };
}
