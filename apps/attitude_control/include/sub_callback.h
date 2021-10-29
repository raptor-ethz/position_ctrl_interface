#pragma once
#include "AttitudeCommandPubSubTypes.h"
#include "quadcopter_msgs/msgs/AttitudeCommand.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::AttitudeCommand attitude_cmd;
} // namespace sub
