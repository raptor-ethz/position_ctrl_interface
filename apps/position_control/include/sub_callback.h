#pragma once

/* subscribers */

#include "QuadPosCmd_msg.h"
#include "QuadPosCmd_msgPubSubTypes.h"

#include "QuadAction_msg.h"
#include "QuadAction_msgPubSubTypes.h"

namespace sub {
cpp_msg::QuadPosCmd_msg pos_cmd;
cpp_msg::QuadAction_msg action_cmd;
} // namespace sub

/* publishers */

#include "QuadStatus_msg.h"
#include "QuadStatus_msgPubSubTypes.h"

namespace pub {
cpp_msg::QuadStatus_msg status_msg;
}
