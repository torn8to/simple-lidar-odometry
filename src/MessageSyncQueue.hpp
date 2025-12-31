#pragma once
#include <deque>
#include <functional>
#include <pair>
#include <rosidl_generator_traits/is_message.hpp>
#include <std_msgs/msg/Header.hpp>
#include <type_traits>
#include <vector>

static_assert(rosidl_generator_traits::is_message_v<MSG_TYPE1>,
              "MSG_TYPE1 needs to be a ros2 message type");
static_assert(rosidl_generator_traits::is_message_v<MSG_TYPE2>,
              "MSG_TYPE2 needs to be a ros2 message type");
static_assert(
    std::is_same_v<decltype(MSG_TYPE1::Header), std_msgs::msg::Header>,
    "MSG_TYPE1 must have a header");
static_assert(
    std::is_same_v<decltype(MSG_TYPE2::Header), std_msgs::msg::Header>,
    "MSG_TYPE2 must have a header");

public:
MsgDeque(double laggard_time_window = 10.0)
    : laggard_time_window_(laggard_time_window) {}

void addToMasterQueue(MSG_TYPE1::SharedPtr msg) { msg1_queue.push_back(msg); }

void addToSlaveQueue(MSG_TYPE2 msg) { msg2_queue.push_back(msg); }

bool isApprox(MSG_TYPE1::SharedPtr msg1, MSG_TYPE2::SharedPtr msg2,
              double threshold = 1e-4){
    return (msg1 -> header.timestamp - msg2->header.timestamp).seconds() <
           threshold}

std::optional<MSG_TYPE2::SharedPtr> getCorrespondingMessagesFromQueue(
    const MSG_TYPE1::SharedPtr ref_msg) {
  for (auto it = msg2_queue.begin(); it != msg2_queue.end(); ++it) {
    if (isApprox(*it, ref_msg)) {
      return *it;
    }
  }
  return {};
}

std::vector<std::pair<MSG_TYPE1::SharedPtr, MSG_TYPE2::SharedPtr>>
GetTimeMatchedPairs() {
  std::vector<std::pair<MSG_TYPE1::SharedPtr, MSG_TYPE2::SharedPtr>>
      msg_pairs_vector;
  for (auto it = msg1_queue.begin(); it != msg1_queue.end(); ++it) {
    auto corresponding = getCorrespondingMessagesFromQueue(*it);
    if (corresponding) {
      msg_pairs_vector.push_back({*it, *corresponding});
    }
  }
  msg_pairs_vector.shrink_to_fit();
  return msg_pairs_vector;
}

private:
double laggard_time_window_;
std::deque<MSG_TYPE1::SharedPtr> msg1_queue;
std::deque<MSG_TYPE2::SharedPtr> msg2_queue;
}
;

using OdomeCloudSyncQueue =
    MsgDeque<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
