#pragma once

#include <string>
#include <iostream>

namespace ros_bridge_client
{
    struct Header
    {
        uint32_t seq;
        uint64_t stamp;
        std::string frame_id;
    };

    struct Point
    {
        float x, y, z;
    };

    struct Pose
    {
        float x, y, theta;
    };

    struct PointStamped
    {
        Header header;
        Point point;
    };


}