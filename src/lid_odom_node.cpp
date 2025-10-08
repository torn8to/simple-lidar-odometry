#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <mutex>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "Pipeline.hpp"
#include "Convert.hpp"
#include "tf2_sophus.hpp"
#include "MotionCompensation.hpp"

namespace lid_odom {

class LidarOdometryNode : public rclcpp::Node {
public:
  explicit LidarOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidar_odometry_mapping", options),
    buffer_(this->get_clock())
  {
    //parameter shenanigans
    declare_parameter("publish_transform", true);
    declare_parameter("debug", true);
    declare_parameter("max_distance", 100.0);
    declare_parameter("voxel_factor", 100.0);
    declare_parameter("voxel_resolution_alpha", 1.5);
    declare_parameter("voxel_resolution_beta", 0.5);
    declare_parameter("max_points_per_voxel", 27);
    declare_parameter("odom_downsample", true);
    declare_parameter("map_frame", "lid_odom");
    declare_parameter("child_frame", "base_link");
    declare_parameter("lidar_frame", "lidar_link");
    declare_parameter("base_frame", "base_link");
    declare_parameter("odom_intgration", false);
    declare_parameter("position_covariance", 0.01);
    declare_parameter("orientation_covariance", 0.01);

    declare_parameter("/threshold/initial_threshold",0.1);
    declare_parameter("/threshold/min_motion_threshold",2.0);
    declare_parameter("/threshold/fixed_threshold", 0.3);
    declare_parameter("/registration/num_iterations", 0.5);
    declare_parameter("/registration/convergence", 1e-4);
    // Get parameters
    config.max_distance = get_parameter("max_distance").as_double();
    config.voxel_factor = get_parameter("voxel_factor").as_double();
    config.voxel_resolution_alpha = get_parameter("voxel_resolution_alpha").as_double();
    config.voxel_resolution_beta = get_parameter("voxel_resolution_beta").as_double();
    config.max_points_per_voxel = get_parameter("max_points_per_voxel").as_int();
    //config.imu_integration_enabled = get_parameter("imu_integration_enabled").as_bool();
    config.odom_downsample = get_parameter("odom_downsample").as_bool();


    config.initial_threshold = get_parameter("/threshold/initial_threshold").as_double();
    config.min_motion_threshold = get_parameter("/threshold/min_motion_threshold").as_double();
    config.fixed_threshold = get_parameter("/threshold/fixed_threshold").as_double();
    config.num_iterations = get_parameter("/registration/num_iterations").as_double();
    config.convergence = get_parameter("/registration/convergence").as_double();
  

    odom_frame_id_ = get_parameter("map_frame").as_string();
    child_frame_id_ = get_parameter("child_frame").as_string();
    base_frame_id_ = get_parameter("base_frame").as_string();
    lidar_link_id_ = get_parameter("lidar_frame").as_string();
    publish_transform_ = get_parameter("publish_transform").as_bool();
    debug_ = get_parameter("debug").as_bool();

    position_covariance_ = get_parameter("position_covariance").as_double();
    orientation_covariance_ = get_parameter("orientation_covariance").as_double();
    
    pipeline_ = std::make_unique<cloud::Pipeline>(config);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
    
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", 10, 
      std::bind(&LidarOdometryNode::pointCloudCallback, this, std::placeholders::_1));


    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lid_odom", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_map", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    
    lidar_pose_acquired = false;
    imu_pose_acquired = false;

    linear_velocity_ = Eigen::Vector3d::Zero();
    angular_velocity_ = Eigen::Vector3d::Zero();
    
    has_first_imu_message = false;
    has_last_lidar_time = false;
    
    last_lidar_pose_ = Sophus::SE3d();
    pose_diff_ = Sophus::SE3d();

    RCLCPP_INFO(get_logger(),"Lidar Odometry Node initialized");
  }

private:

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    rclcpp::Time current_time = msg->header.stamp;
    // set pose biasses
    std::vector<Eigen::Vector3d> points = cloud::convertMsgToCloud(msg);
    std::vector<double> timestamps = cloud::extractTimestampsFromCloudMsg(msg);

    if(!lidar_pose_acquired){
      RCLCPP_WARN(get_logger(), "Lidar pose not acquired trying to reacquire");
      try{
        geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(lidar_link_id_,
                                                           base_frame_id_, tf2::TimePointZero);
        lidar_pose_rel_to_base_ = tf2::transformToSophus(transform_stamped);
        lidar_pose_acquired = true;
      }

      catch(const std::exception & e){
        RCLCPP_ERROR(get_logger(), "Failed to lookup transform from %s to %s: %s", base_frame_id_.c_str(), lidar_link_id_.c_str(), e.what());
        return;
      }
    }
    
    if (points.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty point cloud, skipping");
      return;
    }

    std::vector<Eigen::Vector3d> transformed_points = transformPointCloud(points, lidar_pose_rel_to_base_);
    std::vector<Eigen::Vector3d> unskewed_points;
    
    if (!timestamps.empty() && timestamps.size() == points.size()) {
      unskewed_points = cloud::motionDeSkew(transformed_points, timestamps, pose_diff_);
    } else {
      unskewed_points = transformed_points;
    }
    
    std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>> result = pipeline_->odometryUpdate(unskewed_points, last_lidar_pose_, false);
    Sophus::SE3d updated_pose = std::get<0>(result);

    publishOdometry(msg->header.stamp, pipeline_->position());
    if (debug_){
      this->publishDebug(std::get<1>(result));
    }

    last_lidar_time_ = current_time;
  }


  void publishDebug(const std::vector<Eigen::Vector3d> &cloud){
    if(!pipeline_->mapEmpty()){
      std::vector<Eigen::Vector3d> map  = pipeline_->getMap();
      std::vector<Eigen::Vector3d> map_relative_pose = transformPointCloud(map,pipeline_->position().inverse());
      sensor_msgs::msg::PointCloud2::SharedPtr map_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      map_msg->header.stamp = this->now();
      map_msg->header.frame_id = base_frame_id_;
      cloud::convertCloudToMsg(map_relative_pose, map_msg);
      map_pub_->publish(*map_msg);
    }
    //sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    //cloud_msg->header.stamp = this->now();
    //cloud_msg->header.frame_id = odom_frame_id_;
    //cloud::convertCloudToMsg(cloud, cloud_msg);
    //cloud_pub_->publish(*cloud_msg);
  }


  std::vector<Eigen::Vector3d> transformPointCloud(const std::vector<Eigen::Vector3d> &points,
                                                          const Sophus::SE3d &pose){
    std::vector<Eigen::Vector3d> transformed_points;
    transformed_points.reserve(points.size());
    for(Eigen::Vector3d const &point: points){
      transformed_points.push_back(pose * point);
    }
    return transformed_points;
  }
  
  
  void publishOdometry(const rclcpp::Time & timestamp, const Sophus::SE3d & pose)
  {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = timestamp;
    odom_msg->header.frame_id = odom_frame_id_;
    odom_msg->child_frame_id = child_frame_id_;
    odom_msg->pose.pose.position.x = pose.translation().x();
    odom_msg->pose.pose.position.y = pose.translation().y();
    odom_msg->pose.pose.position.z = pose.translation().z();
    odom_msg->pose.covariance.fill(0.0);
    odom_msg->pose.covariance[0] = position_covariance_;
    odom_msg->pose.covariance[7] = position_covariance_;
    odom_msg->pose.covariance[14] = position_covariance_;
    odom_msg->pose.covariance[21] = orientation_covariance_;
    odom_msg->pose.covariance[28] = orientation_covariance_;
    odom_msg->pose.covariance[35] = orientation_covariance_;
    Eigen::Quaterniond quat(pose.rotationMatrix());
    odom_msg->pose.pose.orientation.w = quat.w();
    odom_msg->pose.pose.orientation.x = quat.x();
    odom_msg->pose.pose.orientation.y = quat.y();
    odom_msg->pose.pose.orientation.z = quat.z();
    odom_pub_->publish(std::move(odom_msg));
    if (publish_transform_){
      geometry_msgs::msg::TransformStamped transform_stamped = tf2::sophusToTransform(pose);
      transform_stamped.header.frame_id = odom_frame_id_;
      transform_stamped.child_frame_id = child_frame_id_;
      transform_stamped.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform_stamped);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::SubscriptionOptions imu_subscription_options_;

  std::vector<std::pair<Sophus::SE3d, double>> imu_pose_diff_queue;

  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;
  
  Eigen::Vector3d linear_acceleration_bias_;
  Eigen::Vector3d angular_velocity_bias_;
  Eigen::Vector3d gravity_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  float position_covariance_, orientation_covariance_;
  cloud::PipelineConfig config;
  std::unique_ptr<cloud::Pipeline> pipeline_;

  rclcpp::Time last_time_;  
  rclcpp::Time last_lidar_time_;  
  Sophus::SE3d lidar_pose_rel_to_base_;
  Sophus::SE3d imu_pose_rel_to_base_;

  Sophus::SE3d pose_diff_; // latest fused pose between imu and lidar_data
  Sophus::SE3d current_lidar_pose; // latest fused pose between imu and lidar_data
  Sophus::SE3d last_lidar_pose_;

  std::string odom_frame_id_;
  std::string child_frame_id_;
  std::string lidar_link_id_;
  std::string imu_frame_id_;
  std::string base_frame_id_;

  bool has_first_imu_message;
  bool has_last_lidar_time;
  bool imu_pose_acquired;
  bool lidar_pose_acquired;
  bool publish_transform_;
  bool debug_;
};

} // namespace lid_odom


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto lid_odom_node = std::make_shared<lid_odom::LidarOdometryNode>();
  executor.add_node(lid_odom_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
