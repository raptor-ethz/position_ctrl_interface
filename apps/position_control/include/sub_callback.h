#pragma once

#include "PosCmd_msg.h"
#include "PosCmd_msgPubSubTypes.h"

#include "Header_msg.h"
#include "Header_msgPubSubTypes.h"
// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::PosCmd_msg pos_cmd;
cpp_msg::Header_msg px4_cmd;
} // namespace sub

namespace pub {
cpp_msg::Header_msg error_msg;
}
