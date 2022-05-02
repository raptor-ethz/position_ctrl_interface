#include "takeoff.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

/* FUNCTION DECLARATIONS */
// thrust-throttle relation (linear)
float thrust_to_throttle(float thrust)
{
    if (thrust > 4 * takeoff_params::max_motor_thrust)
    {
        return 1;
    }
    if (thrust < 0)
    {
        return 0;
    }
    return (0.02394 * thrust + 0.1644);
}

bool takeoff(const mavsdk::Offboard &offboard, const mavsdk::Telemetry &telemetry, float altitude, float yaw)
{
    std::cout << "Taking off...\n";

    /* SEND OFFBOARD ONCE BEFORE STARTING (otherwise it will be rejected) */
    Offboard::Attitude att_cmd{};
    offboard.set_attitude(att_cmd);

    /* STARTING OFFBOARD */
    Offboard::Result offboard_result = offboard.start();
    std::cerr << "Offboard Result: " << offboard_result << '\n';

    /* INITIALIZE VARIABLES */

    // reference values
    Eigen::Vector3f pos_ref(0, 0, altitude);
    Eigen::Vector3f vel_ref(0, 0, 0);
    Eigen::Vector3f acc_ref(0, 0, 0);
    Eigen::Vector3f x_b_ref(0, 0, 0);
    Eigen::Vector3f y_b_ref(0, 0, 0);
    Eigen::Vector3f z_b_ref(0, 0, 0);
    Eigen::Matrix3f body_frame_ref;
    Eigen::Vector3f euler_ref(0, 0, 0);
    float yaw_ref = yaw;

    // current states
    Eigen::Vector3f pos(0, 0, 0);                  // position
    Eigen::Vector3f vel(0, 0, 0);                  // velocity
    Eigen::Quaternion<float> att_quat(0, 0, 0, 0); // attitude quaternion
    Eigen::Vector3f att_euler(0, 0, 0);            // attitude euler angles
    Eigen::Matrix3f body_frame;                    // rotation matrix

    // controller errors
    Eigen::Vector3f pos_p_error(0, 0, 0);
    Eigen::Vector3f vel_p_error(0, 0, 0);
    Eigen::Vector3f vel_p_error_last(0, 0, 0);
    Eigen::Vector3f vel_i_error(0, 0, 0);
    Eigen::Vector3f vel_d_error(0, 0, 0);

    float t = 0;
    const float T_s_sec = float(takeoff_params::T_s) / 1000.0;

    for (int i = 0;; i++) // control loop at 50Hz
    {
        // indices -> real-time
        t = float(i * takeoff_params::T_s) / 1000.0;

        /* CURRENT STATE */
        // current position
        pos(0) = telemetry.position_velocity_ned().position.north_m;
        pos(1) = telemetry.position_velocity_ned().position.east_m;
        pos(2) = -telemetry.position_velocity_ned().position.down_m;
        // current velocity
        vel(0) = telemetry.position_velocity_ned().velocity.north_m_s;
        vel(1) = telemetry.position_velocity_ned().velocity.east_m_s;
        vel(2) = -telemetry.position_velocity_ned().velocity.down_m_s;
        // current orientation (euler angles)
        att_euler(0) = telemetry.attitude_euler().roll_deg;
        att_euler(1) = telemetry.attitude_euler().pitch_deg;
        att_euler(2) = telemetry.attitude_euler().yaw_deg;
        // current orientation (quaternion)
        att_quat.w() = telemetry.attitude_quaternion().w;
        att_quat.x() = telemetry.attitude_quaternion().x;
        att_quat.y() = telemetry.attitude_quaternion().y;
        att_quat.z() = telemetry.attitude_quaternion().z;
        //  body frame (rotation matrix)
        body_frame = att_quat.toRotationMatrix();

        /* POSITION CONTROLLER */
        // proportional position error
        pos_p_error = pos_ref - pos;
        // desired velocity
        vel_ref(0) = takeoff_params::P_pos_XY * pos_p_error(0);
        vel_ref(1) = takeoff_params::P_pos_XY * pos_p_error(1);
        vel_ref(2) = takeoff_params::P_pos_Z * pos_p_error(2); // different gain for Z-error

        // check maximum velocities and constrain.
        if (vel_ref(0) > takeoff_params::max_vel_XY)
        {
            vel_ref(0) = takeoff_params::max_vel_XY;
        }
        if (vel_ref(0) < -takeoff_params::max_vel_XY)
        {
            vel_ref(0) = -takeoff_params::max_vel_XY;
        }
        if (vel_ref(1) > takeoff_params::max_vel_XY)
        {
            vel_ref(1) = takeoff_params::max_vel_XY;
        }
        if (vel_ref(1) < -takeoff_params::max_vel_XY)
        {
            vel_ref(1) = -takeoff_params::max_vel_XY;
        }
        if (vel_ref(2) > takeoff_params::max_vel_Z_UP)
        {
            vel_ref(2) = takeoff_params::max_vel_Z_UP;
        }
        if (vel_ref(2) < -takeoff_params::max_vel_Z_DOWN)
        {
            vel_ref(2) = -takeoff_params::max_vel_Z_DOWN;
        }

        /* VELOCITY CONTROLLER */
        // last proportional velocity error
        vel_p_error_last = vel_p_error;
        // proportional velocity error
        vel_p_error = vel_ref - vel;
        // integrative velocity error
        vel_i_error += vel_p_error * T_s_sec;
        // derivative velocity error
        vel_d_error = (vel_p_error - vel_p_error_last) / T_s_sec;
        // desired acceleration
        acc_ref(0) = takeoff_params::P_vel_XY * vel_p_error(0) +
                     takeoff_params::I_vel_XY * vel_i_error(0) +
                     takeoff_params::D_vel_XY * vel_d_error(0);
        acc_ref(1) = takeoff_params::P_vel_XY * vel_p_error(1) +
                     takeoff_params::I_vel_XY * vel_i_error(1) +
                     takeoff_params::D_vel_XY * vel_d_error(1);
        acc_ref(2) = takeoff_params::P_vel_Z * vel_p_error(2) +
                     takeoff_params::I_vel_Z * vel_i_error(2) +
                     takeoff_params::D_vel_Z * vel_d_error(2); // different gain for Z-error

        /* CONVERSION TO ANGLES AND THRUST */
        // add gravitational acceleration
        acc_ref(2) = acc_ref(2) - takeoff_params::g;

        // y-vector of global coordinte system turned around yaw_ref
        Eigen::Vector3f y_c(-std::sin(yaw_ref), std::cos(yaw_ref), 0);

        // find reference body frame. For more info see:
        // (https://github.com/uzh-rpg/rpg_quadrotor_control/blob/master/documents/theory_and_math/theory_and_math.pdf)
        z_b_ref = acc_ref;
        z_b_ref.normalize();
        x_b_ref = y_c.cross(z_b_ref);
        x_b_ref.normalize();
        y_b_ref = z_b_ref.cross(x_b_ref);

        // put reference body frame vectors into a matrix
        body_frame_ref.col(0) = x_b_ref;
        body_frame_ref.col(1) = y_b_ref;
        body_frame_ref.col(2) = z_b_ref;

        // calculate euler angles from rotation matrix
        euler_ref = body_frame_ref.eulerAngles(0, 1, 2);

        // project thurst onto body frame z-axis
        float acc_proj_z_b = acc_ref.dot(body_frame.col(2));
        float thrust_ref = (acc_proj_z_b)*takeoff_params::quadcopter_mass; // F=M*a

        float throttle_ref = thrust_to_throttle(thrust_ref);

        /* COMMANDS TO PX4 */
        att_cmd.roll_deg = -euler_ref(0) * (180.0 / M_PI);
        att_cmd.pitch_deg = -euler_ref(1) * (180.0 / M_PI);
        att_cmd.yaw_deg = -euler_ref(2) * (180.0 / M_PI);
        att_cmd.thrust_value = throttle_ref;
        offboard.set_attitude(att_cmd);

        /*CHECK IF TARGET HAS BEEN REACHED OR TIMEOUT */
        if (pos(2) > 0.99 * altitude || t > takeoff_params::timeout)
        {
            break;
        }
        /* SLEEP */
        sleep_for(milliseconds(takeoff_params::T_s)); // 50Hz
    }
    return true;
}