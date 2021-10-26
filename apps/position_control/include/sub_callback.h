#pragma once
#include "QuadPositionCmdPubSubTypes.h"
#include "quadcopter_msgs/msgs/QuadPositionCmd.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::QuadPositionCmd pos_cmd;
} // namespace sub

inline void DDSSubscriber::SubListener::on_data_available(
    eprosima::fastdds::dds::DataReader *reader) {
  eprosima::fastdds::dds::SampleInfo info;

  if (reader->take_next_sample(&sub::pos_cmd, &info) ==
      ReturnCode_t::RETCODE_OK) {
    if (info.valid_data) {

      { // Protection against race condition using mutex
        std::unique_lock<std::mutex> lock(m);

        // Print your structure data here.
        std::cout << "Sample received" << std::endl;
        // Set flag when data received
        new_data = true;
      }

      cv.notify_one();
    }
  }
}
