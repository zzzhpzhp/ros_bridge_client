#include <iostream>
#include <string>
#include <thread>
#include <iostream>
#include <string.h>
#include <zmq.h>
#include <jsoncpp/json/json.h>

// http://docs.ros.org/jade/api/angles/html/namespaceangles.html
#include <angles/angles.h>

// geometry_msgs
// http://docs.ros.org/api/geometry_msgs/html/index-msg.html
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/Polygon.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Pose2D.h>
//#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/Twist.h>

// nav_msgs
// http://docs.ros.org/kinetic/api/nav_msgs/html/index-msg.html
//#include <nav_msgs/Path.h>
//#include <nav_msgs/MapMetaData.h>
//#include <nav_msgs/OccupancyGrid.h>
//#include <nav_msgs/Odometry.h>

// sensor_msgs
// http://docs.ros.org/jade/api/sensor_msgs/html/index-msg.html
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>

// visualization_msgs
// http://docs.ros.org/api/visualization_msgs/html/index-msg.html
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

// tf
// http://docs.ros.org/en/api/tf/html/index-msg.html
//#include <tf/tfMessage.h>

#include "ros_bridge_client.h"
#include <csignal>
#include "msg_decoder.h"
#include "typedefine.h"


ros_bridge_client::ROSBridgeClient rbs_;
std::atomic_bool is_running_{true};

void signalHandler( int signum )
{
    std::cout << ">>>>>>>>>>>>>>> Received Signal " << signum << " <<<<<<<<<<<<<<<<<"<< std::endl;
    if (signum == SIGINT)
    {
        is_running_.store(false);
        rbs_.stop();
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
    signal(SIGINT, signalHandler);
    signal(SIGABRT, signalHandler);
    std::cout << "ROS Bridge Server Start" << std::endl;

    if (!rbs_.initialize())
    {
        std::cerr << "Ros bridge initialize failed." << std::endl;
        return 0;
    }

    rbs_.setTopicCallback("/cmd_vel", CmdVelCallback);
    rbs_.setTopicCallback("/move_base_simple/goal", GoalPoseCallback);
    rbs_.start();

    int test = 0;
    float range = 5.0f;

    while (is_running_.load())
    {

        rbs_.publishPoint("/robot/start_pose", "map", GetRandom(range), GetRandom(range));
        rbs_.publishPoint("/robot/goal_pose", "map",GetRandom(range), GetRandom(range));

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
        rbs_.publishPath("/path", "map", points);

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
        rbs_.publishPoints("/points_test", "map", points, 0.2, 0.5, 0.0, 1.0, 1.0, 0.5);

        rbs_.publishTf("/tf", "map", "odom", 1.0f, 1.0f, 0.0f);
        rbs_.publishTf("/tf", "odom", "base_frame", 1.0f, -2.0f, 3.14f);

        rbs_.publishPose("/pose_test", "map", GetRandom(range), GetRandom(range), GetRandom(3.14));

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
        rbs_.publishPoses("/poses_test", "map", poses);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }

    return 0;
}
