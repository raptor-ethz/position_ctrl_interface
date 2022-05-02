#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <future>

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// constants
namespace takeoff_params
{
    const float P_vel_XY = 1.4;
    const float I_vel_XY = 2.0;
    const float D_vel_XY = 0.0;
    const float P_vel_Z = 4.0;
    const float I_vel_Z = 2.0;
    const float D_vel_Z = 0.0;
    const float P_pos_XY = 0.95;
    const float P_pos_Z = 0.95;
    const float max_vel_Z_DOWN = 1.0;
    const float max_vel_Z_UP = 3.0;
    const float max_vel_XY = 12.0;
    const int T_s = 20;
    const float g = 9.81;
    const float quadcopter_mass = 1.5;
    const float max_motor_thrust = 8.9764;
    const float timeout = 20;
} // namespace params

bool takeoff(const mavsdk::Offboard &offboard, const mavsdk::Telemetry &telemetry, float altitude, float yaw);