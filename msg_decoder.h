#pragma once

// c++
// http://www.cplusplus.com/reference/
//#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
//#include <tuple>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
//#include <codecvt>
//#include <complex>
//#include <exception>
#include <functional>
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
#include <jsoncpp/json/json.h>

namespace ros_bridge_client
{
    inline void decodeCmdVel(Json::Value &msg, float &velocity, float &angular)
    {
        velocity = msg["velocity"].asFloat();
        angular = msg["angular"].asFloat();
    }

    inline void decodeGoalPose(Json::Value &msg, float &x, float &y, float &theta, std::string &frame_id)
    {
        x = msg["x"].asFloat();
        y = msg["y"].asFloat();
        theta = msg["theta"].asFloat();
        frame_id = msg["header"]["frame_id"].asString();
    }
}