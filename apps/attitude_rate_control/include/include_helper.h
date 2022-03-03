#pragma once

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

// MAVSDK

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Fastdds
#include "domain_participant.h"
#include "subscriber.h"
#include "sub_callback.h"