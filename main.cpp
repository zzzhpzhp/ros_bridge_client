#include <iostream>
#include <string>
#include <thread>
#include <jsoncpp/json/json.h>

// http://docs.ros.org/jade/api/angles/html/namespaceangles.html
#include <angles/angles.h>

#include "ros_bridge_client.h"
#include <csignal>
#include "msg_decoder.h"
#include "typedefine.h"

ros_bridge_client::ROSBridgeClient rbc;
std::atomic_bool is_running{true};

void SignalHandler(int signum )
{
    std::cout << ">>>>>>>>>>>>>>> Received Signal " << signum << " <<<<<<<<<<<<<<<<<"<< std::endl;
    if (signum == SIGINT)
    {
        is_running.store(false);
        rbc.stop();
    }
}

void CmdVelCallback(Json::Value& msg)
{
    std::cout << "recv " << msg["topic_name"] << std::endl;
    float vel, ang;
    ros_bridge_client::decodeCmdVel(msg, vel, ang);
    std::cout << "vel ang " << vel << " " << ang << std::endl;
}

void GoalPoseCallback(Json::Value& msg)
{
    float x, y, theta;
    std::string frame_id;
    ros_bridge_client::decodeGoalPose(msg, x, y, theta, frame_id);
    std::cout << "goal " << x << " " << y << " " << theta << " " << frame_id << std::endl;
}

inline float GetRandom(float max_val)
{
    float res = rand();
    res = res / RAND_MAX;
    if (rand() > RAND_MAX / 2)
    {
        res = -res;
    }
    return res * max_val;
}

int main()
{
    signal(SIGINT, SignalHandler);
    signal(SIGABRT, SignalHandler);
    std::cout << "ROS Bridge Server Start" << std::endl;

    if (!rbc.initialize())
    {
        std::cerr << "Ros bridge initialize failed." << std::endl;
        return 0;
    }

    rbc.setTopicCallback("/cmd_vel", CmdVelCallback);
    rbc.setTopicCallback("/move_base_simple/goal", GoalPoseCallback);
    rbc.start();

    int test = 0;
    float range = 5.0f;

    while (is_running.load())
    {
        rbc.publishPoint("/point_test", "map", GetRandom(range), GetRandom(range));

        // 路径消息测试
        std::vector<ros_bridge_client::Point> points;
        ros_bridge_client::Point p;
        p.x = GetRandom(range);
        p.y = GetRandom(range);
        points.emplace_back(p);
        p.x = GetRandom(range);
        p.y = GetRandom(range);
        points.emplace_back(p);
        p.x = GetRandom(range);
        p.y = GetRandom(range);
        points.emplace_back(p);
        rbc.publishPath("/path_test", "map", points);

        // 点集消息测试
        points.clear();
        p.x = -1.0;
        p.y = -2.0;
        points.emplace_back(p);
        p.x = -2.0;
        p.y = -2.0;
        points.emplace_back(p);
        p.x = -1.0;
        p.y = -3.0;
        points.emplace_back(p);
        p.x = -2.0;
        p.y = -3.0;
        points.emplace_back(p);
        rbc.publishPoints("/points_test", "map", points, 0.2, 0.5, 0.0, 1.0, 1.0, 0.5);

        // 条带消息测试
        points.clear();
        p.x = 1.0;
        p.y = -1.0;
        points.emplace_back(p);
        p.x = 2.0;
        p.y = -2.0;
        points.emplace_back(p);
        p.x = 1.0;
        p.y = -3.0;
        points.emplace_back(p);
        p.x = 4.0;
        p.y = -4.0;
        points.emplace_back(p);
        p.x = 2.0;
        p.y = -5.0;
        points.emplace_back(p);
        rbc.publishLineStrip("/line_strip_test", "map", points, 0.1, 1.0, 0.0, 1.0, 0.6);


        // 多线条消息测试
        // 点数必须为奇数
        points.clear();
        p.x = -1.0;
        p.y = 1.0;
        points.emplace_back(p);
        p.x = -2.0;
        p.y = 2.0;
        points.emplace_back(p);
        p.x = -1.0;
        p.y = 3.0;
        points.emplace_back(p);
        p.x = -4.0;
        p.y = 4.0;
        points.emplace_back(p);
        p.x = -5.0;
        p.y = 4.0;
        points.emplace_back(p);
        p.x = -5.0;
        p.y = 1.0;
        points.emplace_back(p);
        rbc.publishLineList("/line_list_test", "map", points, 0.1, 0.5, 0.4, 1.0, 0.6);

        // 多边形消息测试
        points.clear();
        p.x = 2.0;
        p.y = 2.0;
        points.emplace_back(p);
        p.x = 3.0;
        p.y = 0.0;
        points.emplace_back(p);
        p.x = 2.0;
        p.y = -2.0;
        points.emplace_back(p);
        p.x = -2.0;
        p.y = -2.0;
        points.emplace_back(p);
        p.x = -2.0;
        p.y = 2.0;
        points.emplace_back(p);
        p.x = 2.0;
        p.y = 2.0;
        points.emplace_back(p);
        rbc.publishPolygon("/polygon_test", "map", points);

        // tf消息测试
        rbc.publishTf("/tf", "map", "odom", 1.0f, 1.0f, 0.0f);
        rbc.publishTf("/tf", "odom", "base_frame", 1.0f, -2.0f, 3.14f);

        // 单位姿消息测试
        rbc.publishPose("/pose_test", "map", GetRandom(range), GetRandom(range), GetRandom(3.14));

        // 多位姿消息测试
        std::vector<ros_bridge_client::Pose> poses;
        ros_bridge_client::Pose pose;
        pose.x = 1.0;
        pose.y = 1.0;
        pose.theta = 0.5;
        poses.emplace_back(pose);
        pose.x = 2.0;
        pose.y = 2.0;
        pose.theta = 1.0;
        poses.emplace_back(pose);
        pose.x = 3.0;
        pose.y = 3.0;
        pose.theta = 1.5;
        poses.emplace_back(pose);
        rbc.publishPoses("/poses_test", "map", poses);

        // 占据栅格地图消息测试
        std::vector<uint8_t> map;
        map.emplace_back(0);
        map.emplace_back(30);
        map.emplace_back(60);
        map.emplace_back(90);
        map.emplace_back(110);
        map.emplace_back(252);
        map.emplace_back(253);
        map.emplace_back(254);
        map.emplace_back(255);
        rbc.publishOccupancyGrid("/map_test", "map", 3, 3, 0.05f, 1.0f, 1.0f, angles::from_degrees(0.0f), map);

        // 激光消息测试
        std::vector<float> ranges, intensities;
        ranges.emplace_back(3.0);
        ranges.emplace_back(3.0);
        ranges.emplace_back(3.0);
        ranges.emplace_back(3.0);
        ranges.emplace_back(3.0);
        ranges.emplace_back(3.0);
        intensities.emplace_back(1.0);
        intensities.emplace_back(1.0);
        intensities.emplace_back(1.0);
        intensities.emplace_back(1.0);
        intensities.emplace_back(1.0);
        intensities.emplace_back(1.0);
        rbc.publishScan("/scan_test", "map", 0.006, 0.001,
                        angles::from_degrees(60), angles::from_degrees(180),
                        angles::from_degrees(-180), 5.0f, 0.05f, ranges, intensities);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
