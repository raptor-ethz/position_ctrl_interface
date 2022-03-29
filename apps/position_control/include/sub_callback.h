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

#include "Mocap_msg.h"
#include "Mocap_msgPubSubTypes.h"
#include "QuadFeedback_msg.h"
#include "QuadFeedback_msgPubSubTypes.h"
#include "QuadStatus_msg.h"
#include "QuadStatus_msgPubSubTypes.h"

namespace pub {
cpp_msg::QuadFeedback_msg feedback;
cpp_msg::Mocap_msg mocap;
} // namespace pub
