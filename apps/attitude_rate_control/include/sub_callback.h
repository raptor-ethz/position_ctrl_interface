#pragma once
#include "QuadAttitudeRateCommandPubSubTypes.h"
#include "quadcopter_msgs/msgs/QuadAttitudeRateCommand.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::QuadAttitudeRateCommand attitude_cmd{};
} // namespace sub
