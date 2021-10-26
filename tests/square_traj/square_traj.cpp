#include "PositionPubSubTypes.h"
#include "default_participant.h"
#include "default_publisher.h"
#include "sensor_msgs/msgs/Mocap.h"
#include <chrono>
#include <cstdlib>
#include <future>

int main() {
  // Message
  Position position{};

  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "quare_traj_publisher");

  // Create publisher with msg type
  DDSPublisher position_pub(PositionPubSubType(), "pos_cmd", dp.participant());

  // Initialize publisher
  position_pub.init();

  // Go to center
  position.z(1);
  position_pub.publish(position);
  // Delay for quad to catch up
  std::this_thread::sleep_for(std::chrono::seconds(5));

  position.x(0.5);
  position_pub.publish(position);
  // Delay for quad to catch up
  std::this_thread::sleep_for(std::chrono::seconds(4));

  position.y(0.5);
  position_pub.publish(position);
  // Delay for quad to catch up
  std::this_thread::sleep_for(std::chrono::seconds(4));

  position.x(-0.5);
  position_pub.publish(position);
  // Delay for quad to catch up
  std::this_thread::sleep_for(std::chrono::seconds(4));

  position.x(0);
  position.y(0);
  position_pub.publish(position);
  // Delay for quad to catch up
  std::this_thread::sleep_for(std::chrono::seconds(4));
}
