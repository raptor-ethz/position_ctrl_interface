#include "include_helper.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

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

// Does Offboard control using NED co-ordinates.
bool offb_ctrl_ned(mavsdk::Offboard &offboard, DDSSubscriber &cmd_sub) {
  std::cout << "Starting Offboard velocity control in NED coordinates\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  const Offboard::VelocityNedYaw stay{};
  offboard.set_velocity_ned(stay);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }

  std::cout << "Offboard started\n";
  std::cout << "Turn to face East\n";

  // Go up
  std::cout << "Go up 0.5 m east\n";
  Offboard::PositionNedYaw position_msg{};
  position_msg.down_m = -2.0f;

  offboard.set_position_ned(position_msg);

  for (;;) {

    // Blocks until new data is available
    // cmd_sub.listener.wait_for_data();

    position_msg.north_m = -sub::st.x();
    position_msg.east_m = -sub::st.y();
    position_msg.down_m = -sub::st.z();

    offboard.set_position_ned(position_msg);

    sleep_for(seconds(1));
  }

  return true;
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
  DDSSubscriber cmd_sub(PositionPubSubType(), "pos_cmd", dp.participant());

  // Intiailize fastdds subscriber
  cmd_sub.init();
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

  const auto arm_result = action.arm();
  if (arm_result != Action::Result::Success) {
    std::cerr << "Arming failed: " << arm_result << '\n';
    return 1;
  }
  std::cout << "Armed\n";

  const auto takeoff_result = action.takeoff();
  if (takeoff_result != Action::Result::Success) {
    std::cerr << "Takeoff failed: " << takeoff_result << '\n';
    return 1;
  }

  sleep_for(seconds(8));

  //  using local NED co-ordinates
  if (!offb_ctrl_ned(offboard, cmd_sub)) {
    return 1;
  }

  const auto land_result = action.land();
  if (land_result != Action::Result::Success) {
    std::cerr << "Landing failed: " << land_result << '\n';
    return 1;
  }

  // Check if vehicle is still in air
  while (telemetry.in_air()) {
    std::cout << "Vehicle is landing...\n";
    sleep_for(seconds(1));
  }
  std::cout << "Landed!\n";

  // We are relying on auto-disarming but let's keep watching the telemetry for
  // a bit longer.
  sleep_for(seconds(3));
  std::cout << "Finished...\n";

  return 0;
}