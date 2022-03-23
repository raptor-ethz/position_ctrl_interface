#include "include_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

/////////////////////////////////////////////////////////////////////////////////
// Local position offsets
constexpr static float x_offset = 0.5;
constexpr static float y_offset = 0.5;
/////////////////////////////////////////////////////////////////////////////////

void usage(const std::string &bin_name) {
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk &mavsdk) {
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    }
  });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
    std::cerr << "No autopilot found.\n";
    return {};
  }

  // Get discovered system now.
  return fut.get();
}

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
  DDSSubscriber pos_cmd_sub(idl_msg::PosCmd_msgPubSubType(), &sub::pos_cmd,
                            "pos_cmd", dp.participant());

  DDSSubscriber px4_cmd_sub(idl_msg::Header_msgPubSubType(), &sub::px4_cmd,
                            "px4_commands", dp.participant());

  DDSPublisher px4_status_pub(idl_msg::Header_msgPubSubType(),
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
    px4_cmd_sub.listener->wait_for_data();
    if (sub::px4_cmd.id == "info") {
      // long battery_percent =
      //     (long)(telemetry.battery().remaining_percent * 100.0);
      // std::string local_pos;

      // if (telemetry.health().is_local_position_ok) {
      //   battery_percent += 1000;
      // }
      // if (telemetry.health().is_armable) {
      //   battery_percent += 10000;
      // }
      // pub::error_msg.id = "Quadcopter Status";

      // //"local position is " + local_pos +
      // //                   " and kill switch is " + kill_switch;
      // std::cout << battery_percent << std::endl;
      // pub::error_msg.timestamp = battery_percent;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::px4_cmd.id == "arm") {
      const auto arm_result = action.arm();
      std::cout << arm_result << std::endl;
      // pub::error_msg.id = "Arm Result";
      // pub::error_msg.timestamp = (int)arm_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::px4_cmd.id == "disarm") {
      const auto disarm_result = action.disarm();
      std::cout << disarm_result << std::endl;
      // pub::error_msg.id = "Disarm Result";
      // pub::error_msg.timestamp = (int)disarm_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::px4_cmd.id == "takeoff") {
      const auto takeoff_result = action.takeoff();
      std::cout << takeoff_result << std::endl;
      // pub::error_msg.id = "Takeoff Result";
      // pub::error_msg.timestamp = (int)takeoff_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::px4_cmd.id == "land") {
      const auto land_result = action.land();
      std::cout << land_result << std::endl;
      // pub::error_msg.id = "Land Result";
      // pub::error_msg.timestamp = (int)land_result;
      // px4_status_pub.publish(pub::error_msg);
    }
    if (sub::px4_cmd.id == "offboard") {

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

        if (sub::pos_cmd.header.id == "break") {
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