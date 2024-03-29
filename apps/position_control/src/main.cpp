// include dependencies
#include "include_helper.h"

// include MAVSDK helper functions
#include "MAVSDK_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// Local position offsets
constexpr static float x_offset = 0.5;
constexpr static float y_offset = 0.5;

// std::function<void(const mavlink_message_t &)> homePositionCallback =
//     [](const mavlink_message_t &raw_msg)
// {
//   auto longitude = mavlink_msg_home_position_get_longitude(&raw_msg);
//   std::cout << "Longitude: " << longitude << '\n';
// };

int main(int argc, char **argv)
{
  // check if serial port was given as command line argument
  if (argc != 2)
  {
    usage(argv[0]);
    return 1;
  }

  // create mavsdk instance
  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  /* Fastdds */
  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "pos_ctrl_interface");
  // create subscribers
  DDSSubscriber pos_cmd_sub(idl_msg::QuadPosCmd_msgPubSubType(), &sub::pos_cmd,
                            "pos_cmd", dp.participant());
  DDSSubscriber action_cmd_sub(idl_msg::QuadAction_msgPubSubType(),
                               &sub::action_cmd, "px4_commands",
                               dp.participant());
  // create publisher
  DDSPublisher feedback_pub(idl_msg::QuadFeedback_msgPubSubType(),
                            "px4_status_msgs", dp.participant());

  // check connection success
  if (connection_result != ConnectionResult::Success)
  {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system)
  {
    return 1;
  }

  // Instantiate plugins.
  auto action = Action{system};
  auto offboard = Offboard{system};
  auto telemetry = Telemetry{system};
  // auto mavlinkPassthrough = MavlinkPassthrough{system};
  std::cout << "System is ready\n";

  // check matched
  for (int i = 0;
       !feedback_pub.listener.matched() || !action_cmd_sub.listener->matched();
       ++i)
  {
    std::cout << "Publisher hasn't matched. ";

    if (i == 9)
    {
      std::cerr << "Failed to match a subscriber." << std::endl;
      return 1;
    }

    std::cout << "Retrying in 3 seconds (" << 9 - i << " tries remaining)."
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  // TODO
  bool wait_for_data = true;

  // subscribe to home position
  // mavlinkPassthrough.subscribe_message_async(410, homePositionCallback);

  // main loop
  while (true)
  {
    // wait for new data?
    if (wait_for_data)
    {
      std::cout << "wait for action command" << std::endl;
      action_cmd_sub.listener->wait_for_data();
      std::cout << "received an action command" << std::endl;
    }
    else
    {
      std::cout << "skipped wait for data." << std::endl;
      // reset bool
      wait_for_data = true;
    }

    switch (sub::action_cmd.action)
    {
    case Action_cmd::act_status:
    {
      pub::feedback.feedback = FeedbackType::fb_status;
      pub::feedback.status.battery =
          (int)(telemetry.battery().remaining_percent * 100.0);
      pub::feedback.status.local_position_ok =
          telemetry.health().is_local_position_ok;
      pub::feedback.status.armable = telemetry.health().is_armable;
      std::cout << "publish status" << std::endl;
      feedback_pub.publish(pub::feedback);
      std::cout << "published status" << std::endl;
    }
    break;

    case Action_cmd::act_arm:
    {
      const auto arm_result = action.arm();
      std::cout << arm_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_arm;
      if (arm_result == mavsdk::Action::Result::Success)
      {
        pub::feedback.result = ResultType::res_success;
      }
      else
      {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);

      /* SET HOME POSITION */
      // // set home position to current position [NOT TESTED]
      // mavlink_message_t msg;
      // mavlink_set_home_position_t home_position;
      // home_position.x = telemetry.position_velocity_ned().position.north_m;
      // home_position.y = telemetry.position_velocity_ned().position.east_m;
      // home_position.z = telemetry.position_velocity_ned().position.down_m;
      // mavlink_msg_set_home_position_encode(system->get_system_id(),
      //                                      system->component_ids()[0], &msg,
      //                                      &home_position);
      // // publish mavlink message
      // mavlinkPassthrough.send_message(msg);

      // flight mode code TODO
      // mavlink_set_mode_t flight_mode;
      // flight_mode.base_mode = 2; // alt hold?
      // mavlink_msg_set_mode_encode(system->get_system_id(),
      // system->component_ids()[0], &msg, &flight_mode);
      // mavlinkPassthrough.send_message(msg);
    }
    break;

    case Action_cmd::act_disarm:
    {
      const auto disarm_result = action.disarm();
      std::cout << disarm_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_disarm;
      if (disarm_result == mavsdk::Action::Result::Success)
      {
        pub::feedback.result = ResultType::res_success;
      }
      else
      {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    }
    break;

    case Action_cmd::act_takeoff:
    {
      // old method

      const auto takeoff_result = action.takeoff();
      std::cout << takeoff_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_takeoff;
      if (takeoff_result == mavsdk::Action::Result::Success)
      {
        pub::feedback.result = ResultType::res_success;
      }
      else
      {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);

      // new method
      // float takeoff_x = telemetry.position_velocity_ned().position.north_m;
      // float takeoff_y = telemetry.position_velocity_ned().position.east_m;
      // const bool takeoff_result = takeoff(offboard, telemetry, takeoff_x, takeoff_y, 1.5, 0);
      // pub::feedback.feedback = FeedbackType::fb_takeoff;
      // if (takeoff_result == true)
      // {
      //   pub::feedback.result = ResultType::res_success;
      // }
      // else
      // {
      //   pub::feedback.result = ResultType::res_fail;
      // }
      // feedback_pub.publish(pub::feedback);
    }
    break;

    case Action_cmd::act_land:
    {
      const auto land_result = action.land();
      std::cout << land_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_land;
      if (land_result == mavsdk::Action::Result::Success)
      {
        pub::feedback.result = ResultType::res_success;
      }
      else
      {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    }
    break;

    // TODO (remove?)
    case Action_cmd::act_hover:
    {
      const auto hover_result = action.hold();
      std::cout << hover_result << std::endl;
      pub::feedback.feedback = FeedbackType::fb_hover;
      if (hover_result == mavsdk::Action::Result::Success)
      {
        pub::feedback.result = ResultType::res_success;
      }
      else
      {
        pub::feedback.result = ResultType::res_fail;
      }
      feedback_pub.publish(pub::feedback);
    }
    break;

    case Action_cmd::act_offboard:
    {
      // Send it once before starting offboard, otherwise it will be rejected.
      Offboard::PositionNedYaw position_msg{};
      // position_msg.north_m =
      // telemetry.position_velocity_ned().position.north_m; TODO
      // position_msg.east_m =
      // telemetry.position_velocity_ned().position.east_m;
      position_msg.down_m = -1.5f;
      position_msg.yaw_deg = 0;
      offboard.set_position_ned(position_msg);

      Offboard::Result offboard_result = offboard.start();
      offboard.set_position_ned(position_msg);

      while (true)
      {
        // Blocks until new data is available?
        pos_cmd_sub.listener->wait_for_data_for_ms(200);
        std::cout << "going to pos: (" << sub::pos_cmd.position.x << ", " << sub::pos_cmd.position.y << ", " << sub::pos_cmd.position.z << ") " << std::endl;

        if (sub::pos_cmd.position.x < 0.01 && sub::pos_cmd.position.y < 0.01 && sub::pos_cmd.position.z < 0.01 && sub::pos_cmd.position.x > -0.01 && sub::pos_cmd.position.y > -0.01 && sub::pos_cmd.position.z > -0.01)
        {
          std::cout << "Position command not feasible (0,0,0). I'll just stay at (" << telemetry.position_velocity_ned().position.north_m << ", " << telemetry.position_velocity_ned().position.east_m << ", " << -telemetry.position_velocity_ned().position.down_m << ") " << std::endl;
          position_msg.north_m = telemetry.position_velocity_ned().position.north_m;
          position_msg.east_m = telemetry.position_velocity_ned().position.east_m;
          position_msg.down_m = telemetry.position_velocity_ned().position.down_m;

          position_msg.yaw_deg = telemetry.attitude_euler().yaw_deg;
        }
        else if (sub::pos_cmd.position.x > -0.501 && sub::pos_cmd.position.y > -0.501 && sub::pos_cmd.position.x < -0.499 && sub::pos_cmd.position.y < -0.499 && sub::pos_cmd.position.z < -1.999 && sub::pos_cmd.position.z > -2.001)
        {
          std::cout << "Position command not feasible (0.5,0.5,0). I'll just stay at (" << telemetry.position_velocity_ned().position.north_m << ", " << telemetry.position_velocity_ned().position.east_m << ", " << -telemetry.position_velocity_ned().position.down_m << ") " << std::endl;
          position_msg.north_m = telemetry.position_velocity_ned().position.north_m;
          position_msg.east_m = telemetry.position_velocity_ned().position.east_m;
          position_msg.down_m = telemetry.position_velocity_ned().position.down_m;

          position_msg.yaw_deg = telemetry.attitude_euler().yaw_deg;
        }
        else if (sub::pos_cmd.position.z < 0.005)
        {
          std::cout << "Position command not feasible (0.5,0.5,0). I'll just stay at (" << telemetry.position_velocity_ned().position.north_m << ", " << telemetry.position_velocity_ned().position.east_m << ", " << -telemetry.position_velocity_ned().position.down_m << ") " << std::endl;
          position_msg.north_m = telemetry.position_velocity_ned().position.north_m;
          position_msg.east_m = telemetry.position_velocity_ned().position.east_m;
          position_msg.down_m = telemetry.position_velocity_ned().position.down_m;

          position_msg.yaw_deg = telemetry.attitude_euler().yaw_deg;
        }
        else
        {

          position_msg.north_m = sub::pos_cmd.position.x + x_offset;
          position_msg.east_m = sub::pos_cmd.position.y + y_offset;
          position_msg.down_m =
              -sub::pos_cmd.position.z; // To account for px4 -z coordinate system
                                        // (North-East-Down)
          position_msg.yaw_deg = -sub::pos_cmd.yaw_angle;

          // std::cout << "x:\t" << sub::pos_cmd.position.x << "\t y: \t"
          //           << sub::pos_cmd.position.y << "\t z: \t"
          //           << sub::pos_cmd.position.z << std::endl;
        }

        // terminate offboard?
        if (sub::action_cmd.action != Action_cmd::act_offboard)
        {
          break;
        }

        offboard.set_position_ned(position_msg);
      }
      offboard_result = offboard.stop();
      // don't wait for new data after offboard was terminated
      wait_for_data = false;
    }
    break;

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