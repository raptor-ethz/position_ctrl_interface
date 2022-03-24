#pragma once

#include "QuadPosCmd_msg.h"
#include "QuadPosCmd_msgPubSubTypes.h"

#include "Header_msg.h"
#include "Header_msgPubSubTypes.h"
// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::QuadPosCmd_msg pos_cmd;
cpp_msg::Header_msg px4_cmd;
} // namespace sub

namespace pub {
cpp_msg::Header_msg error_msg;
}
