#include <rclcpp/rclcpp.hpp>
#include "FasterTransformNode.hpp"


int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FasterBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}