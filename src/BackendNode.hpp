#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "MessageSyncQueue.hpp"
#include <SlamPipeline.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

class BackendNode : public rclcpp::Node {
public:
  explicit BackendNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) {
    this->declare_parameter("base_frame", "base_frame");
    this->declare_parameter("map/max_distance", 100.0);
    this->declare_parameter("map/max_points_per_voxel", 100.0);
    this->declare_parameter("map/distance_update", 10.0);
    this->declare_parameter("map/bev_resolution", 0.5);
    this->declare_parameter("map/map_voxel_size", 1.0);
    this->declare_parameter("map/sampling_voxel_size_ratio", 1.00);
    this->declare_parameter("graph/loop_closure_weight", 1.5);
    this->declare_parameter("detector/hamming_distance_threshold", 15);
    this->declare_parameter("detector/self_similarity_threshold", 35);
    this->declare_parameter("detector/splitting_strategy", 0);
    this->declare_parameter("detector/succesful_match_threshold", 25);
    this->declare_parameter("ransac/inlier_threshold", 3.0);
    this->declare_parameter("ransac/num_iterations", 10);
    this->declare_parameter("icp/max_iterations", 100);
    this->declare_parameter("icp/num_threads", 16);

    base_frame_id_ = this->get_parameter("base_frame").as_string();
    SlamPipelineConfig slam_config();
    slam_config.max_range = this->get_parameter("map/max_distance").as_double();
    slam_config.max_points_per_voxel =
        this->get_parameter("map/max_points_per_voxel").as_int();
    slam_config.bev_resolution =
        this->get_parameter("map/bev_resolution").as_double();
    slam_config.map_voxel_resolution =
        this->get_parameter("map/voxel_map_size").as_double();
    slam_config.sampling_voxel_size_resolution =
        this->get_parameter("map/sampling_voxel_size_ratio").as_double();
    slam_config.slam_config.num_ransac_iterations =
        this->get_parameter("ransac/num_iterations").as_int();
    slam_config.inlier_threshold =
        this->get_parameter("ransac/inlier_threshold").as_double();
    slam_config.self_similarity_threshold =
        this->get_parameter("ransac/self_similarity_threshold").as_int();
    slam_config.hamming_distance_threshold =
        this->get_parameter("detector/hamming_distance_thrshold").as_int();
    slam_config.point_match_threshold =
        this->get_parameter("detector/succesful_match_threshold").as_int();
    slam_config.point_match_threshold =
        this->get_parameter("detector/succesful_match_threshold").as_int();
    slam_config.self_similarity_threshold =
        this->get_parameter("detector/self_similarity_threshold").as_int();

    slam_config.icp_iterations =
        this->get_parameter("icp/max_iterations").as_int();
    slam_config.icp_threads = this->get_parameter("icp/num_threads").as_int();

    backend_pipeline_ = SlamPipeline(slam_config);

    point_cloud_sub = this ->create_subscription<sensor_msgs::msg::PointCloud2>(
        '/cloud', 10, std::bind(&BackendNode::pointCloud), std::placeholders::_1));

    odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        '/odom', 10,
        std::bind(&BackendNode::OdometryCallback, std::placeholders::_1));
  }

  srrg_hbst::SplittingStrategy getTreeSplit(int strategy_id) {
    if (stategy_id > 0 && strategy_id < 4) {
      RCLCPP_INFO(get_logger(), "the splitting strategy id is invalid "
                                "defalting to uneven splitting strategy");
      return srrg_hbst::SplittingStrategy::SplitUneven;
    }
    return static_cast<srrg_hbst::SplittingStrategy>(id);
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    msg_sync_queue_.addToSlaveQueue(msg);
  }

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    msg_sync_queue_.addToMasterQueue(msg);
    std::vector<std::pair<nav_msgs::msg::Odometry::SharedPtr,
                          sensor_msgs::msg::PointCloud2::SharedPtr>>
        msgs = msg_sync_queue_.getCorrespondingMessagesFromQueue();
    if (msgs.size() == 0) {
      return;
    }
    if (msgs.size() > 1) {
      std::sort(msgs.begin(), msgs.end(), [](auto first, auto second) {
        return first->header.stamp < second->header.stamp;
      });
    }
    for (auto it = msgs.begin(), it != msgs.end(), ++it) {
      // get sophus and get cloud from msgs
      auto &[odom_ptr, cloud_ptr] = *it;
      Eigen::Vector3d point;
      Eigen::Vector3d position;
      Eigen::Quaterniond rotation;
      tf2::fromMsg(odom_ptr->pose.pose.point, position);
      tf2::fromMsg(odom_ptr->pose.pose.orientation, rotation);
      Sophus::SE3d odom_pose(rotation, position);
      std::vector<Eigen::Vector3d> cloud =
          cloud::convertMsgtoCloud(cloud_msg_ptr);
      pipeline.update(pose, cloud);
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_source_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      odometry_source_Publisher_;

  SlamPipelineConfig backend_pipeline_;
  MsgDeque<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>
      msg_sync_queue_;

  // std::vector<std::string, Sophus::SE3d> transform_cache_;
  std::string base_frame_id_;
}