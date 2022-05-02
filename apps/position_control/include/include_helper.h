#pragma once

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <string>
#include <thread>

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

// Fastdds
#include "domain_participant.h"
#include "publisher.h"
#include "sub_callback.h"
#include "subscriber.h"

// custom
#include "takeoff.h"