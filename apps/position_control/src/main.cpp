// include dependencies
#include "include_helper.h"
// include MAVSDK helper functions
#include "MAVSDK_helper.h"

//#define SIMULATION

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

/////////////////////////////////////////////////////////////////////////////////
// Local position offsets
constexpr static float x_offset = 0.5;
constexpr static float y_offset = 0.5;
/////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  if (argc != 2) {
    usage(argv[0]);
    return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  ////////////////////////////////////////////////////////////////////////
  // Fastdds

  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "pos_ctrl_interface");

  // Create subscriber with msg type
  DDSSubscriber pos_cmd_sub(idl_msg::QuadPosCmd_msgPubSubType(), &sub::pos_cmd,
                            "pos_cmd", dp.participant());

  DDSSubscriber action_cmd_sub(idl_msg::QuadAction_msgPubSubType(),
                               &sub::action_cmd, "px4_commands",
                               dp.participant());

  DDSPublisher feedback_pub(idl_msg::QuadFeedback_msgPubSubType(),
                            "px4_status_msgs", dp.participant());

#ifdef SIMULATION
  std::cout << "THIS CODE IS FOR RUNNING IN A SIMULATION FRAMEWORK"
            << std::endl;
  DDSPublisher mocap_pub(idl_msg::Mocap_msgPubSubType(), "mocap_srl_quad",
                         dp.participant());
#endif

  /////////////////////////////////////////////////////////////////////////////////

  if (connection_result != ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system) {
    return 1;
  }

  // Instantiate plugins.
  auto action = Action{system};
  auto offboard = Offboard{system};
  auto telemetry = Telemetry{system};
  std::cout << "System is ready\n";

  // pub::error_msg.id = "THIS IS A TEST ERROR MESSAGE";
  // px4_status_pub.publish(pub::error_msg);

  // test for health and battery
  //  while (true) {
  //    int battery_percent = (int)(telemetry.battery().remaining_percent *
  //    100.0);

  //   std::string local_pos;
  //   if (telemetry.health().is_local_position_ok) {
  //     local_pos = "ok";
  //   } else {
  //     local_pos = "not ok";
  //   }

  //   std::string kill_switch;
  //   if (telemetry.health().is_armable) {
  //     kill_switch = "disengaged";
  //   } else {
  //     kill_switch = "engaged";
  //   }

  //   std::string health_info =
  //       "local position is " + local_pos + " and kill switch is " +
  //       kill_switch;
  //   std::cout << "battery: " << battery_percent << std::endl;
  // }

#ifdef SIMULATION
  // publish mocap data for reference generator
  for (int i = 0; i < 30; i++) {
    //  xyz
    pub::mocap.position.x =
        telemetry.position_velocity_ned().position.north_m - x_offset;
    pub::mocap.position.y =
        telemetry.position_velocity_ned().position.east_m - y_offset;
    pub::mocap.position.z = -telemetry.position_velocity_ned().position.down_m;
    // rpy
    pub::mocap.orientation.roll = telemetry.attitude_euler().roll_deg;
    pub::mocap.orientation.roll = telemetry.attitude_euler().pitch_deg;
    pub::mocap.orientation.roll = telemetry.attitude_euler().yaw_deg;
    // publish message
    mocap_pub.publish(pub::mocap);
  }
#endif

  // check matched
  for (int i = 0;
       !feedback_pub.listener.matched() || !action_cmd_sub.listener->matched();
       ++i) {
    std::cout << "Publisher hasn't matched. ";

    if (i == 9) {
      std::cerr << "Failed to match a subscriber." << std::endl;
      return 1;
    }

    std::cout << "Retrying in 3 seconds (" << 9 - i << " tries remaining)."
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  while (true) {
    std::cout << "wait for action command" << std::endl;
    action_cmd_sub.listener->wait_for_data();
    std::cout << "received an action command" << std::endl;
    switch (sub::action_cmd.action) {
    case Action_cmd::act_status: {
      pub::feedback.feedback = FeedbackType::fb_status;
      pub::feedback.status.battery =
          (int)(telemetry.battery().remaining_percent * 100.0);
      pub::feedback.status.local_position_ok =
          telemetry.health().is_local_position_ok;
      pub::feedback.status.armable = telemetry.health().is_armable;
      std::cout << "publish status" << std::endl;
      feedback_pub.publish(pub::feedback);
      std::cout << "published status" << std::endl;
    } break;

    case Action_cmd::act_arm: {
      const auto arm_result = action.arm();
      std::cout << arm_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_arm;
      if (arm_result == mavsdk::Action::Result::Success) {
        pub::feedback.result = ResultType::res_success;
      } else {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    } break;

    case Action_cmd::act_disarm: {
      const auto disarm_result = action.disarm();
      std::cout << disarm_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_disarm;
      if (disarm_result == mavsdk::Action::Result::Success) {
        pub::feedback.result = ResultType::res_success;
      } else {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    } break;

    case Action_cmd::act_takeoff: {
      const auto takeoff_result = action.takeoff();
      std::cout << takeoff_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_takeoff;
      if (takeoff_result == mavsdk::Action::Result::Success) {
        pub::feedback.result = ResultType::res_success;
      } else {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    } break;

    case Action_cmd::act_land: {
      const auto land_result = action.land();
      std::cout << land_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_land;
      if (land_result == mavsdk::Action::Result::Success) {
        pub::feedback.result = ResultType::res_success;
      } else {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    } break;

    case Action_cmd::act_hover: {
      const auto hover_result = action.hold();
      std::cout << hover_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_hover;
      if (hover_result == mavsdk::Action::Result::Success) {
        pub::feedback.result = ResultType::res_success;
      } else {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    } break;

    case Action_cmd::act_offboard: {
      // Send it once before starting offboard, otherwise it will be rejected.
      Offboard::PositionNedYaw position_msg{};
      position_msg.down_m = -1.5f;
      offboard.set_position_ned(position_msg);

      Offboard::Result offboard_result = offboard.start();
      offboard.set_position_ned(position_msg);

      while (true) {
        // Blocks until new data is available
        pos_cmd_sub.listener->wait_for_data();

        if (sub::pos_cmd.header.description == "break") {
          break;
        }
        position_msg.north_m = sub::pos_cmd.position.x + x_offset;
        position_msg.east_m = sub::pos_cmd.position.y + y_offset;
        position_msg.down_m =
            -sub::pos_cmd.position.z; // To account for px4 -z coordinate system
                                      // (North-East-Down)
        position_msg.yaw_deg = -sub::pos_cmd.yaw_angle;

        offboard.set_position_ned(position_msg);

        // #ifdef SIMULATION
        //         pub::mocap.header.description = "mocap message";
        //         // xyz
        //         pub::mocap.position.x =
        //             telemetry.position_velocity_ned().position.north_m -
        //             x_offset;
        //         pub::mocap.position.y =
        //             telemetry.position_velocity_ned().position.east_m -
        //             y_offset;
        //         pub::mocap.position.z =
        //             -telemetry.position_velocity_ned().position.down_m;
        //         // rpy
        //         pub::mocap.orientation.roll =
        //         telemetry.attitude_euler().roll_deg;
        //         pub::mocap.orientation.roll =
        //         telemetry.attitude_euler().pitch_deg;
        //         pub::mocap.orientation.roll =
        //         telemetry.attitude_euler().yaw_deg;
        //         // publish message
        //         mocap_pub.publish(pub::mocap);
        // #endif
      }
      offboard_result = offboard.stop();
    } break;

    default:
      break;
    }

    // // old implementation
    // if (sub::action_cmd.action == Action_cmd::status) {
    //   pub::feedback.battery =
    //       (int)(telemetry.battery().remaining_percent * 100.0);

    //   pub::feedback.local_position_ok =
    //       telemetry.health().is_local_position_ok;

    //   pub::feedback.armable = telemetry.health().is_armable;

    //   feedback_pub.publish(pub::feedback);
    // }
    // if (sub::action_cmd.action == Action_cmd::arm) {
    //   const auto arm_result = action.arm();
    //   std::cout << arm_result << std::endl;
    //   // pub::error_msg.id = "Arm Result";
    //   // pub::error_msg.timestamp = (int)arm_result;
    //   // px4_status_pub.publish(pub::error_msg);
    // }
    // if (sub::action_cmd.action == Action_cmd::disarm) {
    //   const auto disarm_result = action.disarm();
    //   std::cout << disarm_result << std::endl;
    //   // pub::error_msg.id = "Disarm Result";
    //   // pub::error_msg.timestamp = (int)disarm_result;
    //   // px4_status_pub.publish(pub::error_msg);
    // }
    // if (sub::action_cmd.action == Action_cmd::takeoff) {
    //   const auto takeoff_result = action.takeoff();
    //   std::cout << takeoff_result << std::endl;
    //   // pub::error_msg.id = "Takeoff Result";
    //   // pub::error_msg.timestamp = (int)takeoff_result;
    //   // px4_status_pub.publish(pub::error_msg);
    // }
    // if (sub::action_cmd.action == Action_cmd::land) {
    //   const auto land_result = action.land();
    //   std::cout << land_result << std::endl;
    //   // pub::error_msg.id = "Land Result";
    //   // pub::error_msg.timestamp = (int)land_result;
    //   // px4_status_pub.publish(pub::error_msg);
    // }
    // if (sub::action_cmd.action == Action_cmd::offboard) {

    //   // Send it once before starting offboard, otherwise it will be
    //   rejected. const Offboard::PositionNedYaw stay{};
    //   offboard.set_position_ned(stay);

    //   Offboard::Result offboard_result = offboard.start();
    //   Offboard::PositionNedYaw position_msg{};
    //   position_msg.down_m = -1.5f;
    //   offboard.set_position_ned(position_msg);
    //   // sleep_for(milliseconds(100));

    //   while (true) {
    //     // Blocks until new data is available
    //     pos_cmd_sub.listener->wait_for_data();

    //     if (sub::pos_cmd.header.description == "break") {
    //       break;
    //     }
    //     position_msg.north_m = sub::pos_cmd.position.x + x_offset;
    //     position_msg.east_m = sub::pos_cmd.position.y + y_offset;
    //     position_msg.down_m =
    //         -sub::pos_cmd.position.z; // To account for px4 -z coordinate
    //         system
    //                                   // (North-East-Down)
    //     position_msg.yaw_deg = -sub::pos_cmd.yaw_angle;

    //     offboard.set_position_ned(position_msg);
    //   }
    //   offboard_result = offboard.stop();
    // }
  }
  return 0;
}