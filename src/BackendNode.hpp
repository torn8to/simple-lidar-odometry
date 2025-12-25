#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <loop_closure_detector/Pipeline.hpp>




class BackendNode : public rclcpp::Node{
  public:
    explicit BackendNode(const rclcpp::NodeOptions & options = rclcpp::Nodeoptions()){

    }



  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_source_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_source_subscriber;


    LoopClosureDetector::Pipeline backend_pipeline_;



    double img_range_;
    double distance_database_add_threshold_;
}