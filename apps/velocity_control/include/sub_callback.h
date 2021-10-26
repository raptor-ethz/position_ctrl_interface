#pragma once
#include "QuadVelocityCmdPubSubTypes.h"
#include "quadcopter_msgs/msgs/QuadVelocityCmd.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::QuadVelocityCmd velocity_cmd;
} // namespace sub
