#pragma once
#include "QuadAttitudeCommandPubSubTypes.h"
#include "quadcopter_msgs/msgs/QuadAttitudeCommand.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::QuadAttitudeCommand attitude_cmd{};
} // namespace sub
