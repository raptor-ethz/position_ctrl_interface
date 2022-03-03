#pragma once

#include "QuadPositionCmdPubSubTypes.h"
#include "quadcopter_msgs/msgs/QuadPositionCmd.h"

#include "HeaderPubSubTypes.h"
#include "std_msgs/msgs/Header.h"
// Subscriber data that needs to be accessed in main
namespace sub {
    cpp_msg::QuadPositionCmd pos_cmd;
    cpp_msg::Header px4_cmd;
} // namespace sub

namespace pub{
    cpp_msg::Header error_msg;
}
