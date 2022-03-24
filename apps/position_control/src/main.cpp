// include dependencies
#include "include_helper.h"
// include MAVSDK helper functions
#include "MAVSDK_helper.h"

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

  DDSPublisher status_pub(idl_msg::QuadStatus_msgPubSubType(),
                          "px4_status_msgs", dp.participant());
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

  while (true) {
    action_cmd_sub.listener->wait_for_data();
    if (sub::action_cmd.action == Action_cmd::status) {
      pub::status_msg.battery =
          (int)(telemetry.battery().remaining_percent * 100.0);

      pub::status_msg.local_position_ok =
          telemetry.health().is_local_position_ok;

      pub::status_msg.armable = telemetry.health().is_armable;

      status_pub.publish(pub::status_msg);
    }
    if (sub::action_cmd.action == Action_cmd::arm) {
      const auto arm_result = action.arm();
      std::cout << arm_result << std::endl;
      // pub::error_msg.id = "Arm Result";
      // pub::error_msg.timestamp = (int)arm_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::action_cmd.action == Action_cmd::disarm) {
      const auto disarm_result = action.disarm();
      std::cout << disarm_result << std::endl;
      // pub::error_msg.id = "Disarm Result";
      // pub::error_msg.timestamp = (int)disarm_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::action_cmd.action == Action_cmd::takeoff) {
      const auto takeoff_result = action.takeoff();
      std::cout << takeoff_result << std::endl;
      // pub::error_msg.id = "Takeoff Result";
      // pub::error_msg.timestamp = (int)takeoff_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::action_cmd.action == Action_cmd::land) {
      const auto land_result = action.land();
      std::cout << land_result << std::endl;
      // pub::error_msg.id = "Land Result";
      // pub::error_msg.timestamp = (int)land_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::action_cmd.action == Action_cmd::offboard) {

      // Send it once before starting offboard, otherwise it will be rejected.
      const Offboard::PositionNedYaw stay{};
      offboard.set_position_ned(stay);

      Offboard::Result offboard_result = offboard.start();
      Offboard::PositionNedYaw position_msg{};
      position_msg.down_m = -1.5f;
      offboard.set_position_ned(position_msg);
      // sleep_for(milliseconds(100));

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
      }
      offboard_result = offboard.stop();
    }
  }
  return 0;
}