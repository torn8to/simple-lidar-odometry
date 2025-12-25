#include <rclcpp/rclcpp.hpp>
#include "LidarOdometryNode.hpp"


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto lid_odom_node = std::make_shared<LidarOdometryNode>();
  executor.add_node(lid_odom_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
