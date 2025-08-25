#pragma once

#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "VoxelMap.hpp"
#include "Registration.hpp"
#include "Threshold.hpp"


namespace cloud {

struct PipelineConfig {
  double max_distance = 100.0;
  double voxel_factor = 100;
  double voxel_resolution_alpha = 1.5;
  double voxel_resolution_beta = 0.5; // beta is recommended to be smaller for better odom updates
  bool imu_integration_enabled = false;
  int max_points_per_voxel = 27;
  int num_threads = 8;
  int num_iterations = 100;
  double convergence = 1e-4;
  bool odom_downsample = true;
  double initial_threshold = 0.5;
  double min_motion_threshold =  0.1;
  int lfu_prune_interval = 20;
};

class Pipeline {
public:
  /**
   * @brief Constructor for the Pipeline class
   * @param config Configuration parameters for the pipeline
   */
  explicit Pipeline(const PipelineConfig &config = PipelineConfig());
  
  /**
   * @brief Destructor for the Pipeline class
   */
  ~Pipeline();

  /**
   * @brief Performs an odometry update using the existing pose
   *
   * This method performs an odometry update using the existing pose and the new cloud.
   * It first performs a voxelization of the new cloud and then performs a nearest neighbor search
   * to find the closest points in the voxel map.
   *
   * @param cloud The new cloud to update odometry
   * @param external_guess External pose guess (optional)
   * @param use_external_guess Whether to use the external pose guess (optional)
   * @return A tuple containing the updated pose and the voxelized cloud
   */
  std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>> odometryUpdate(
      std::vector<Eigen::Vector3d> &cloud,
      const Sophus::SE3d &external_guess = Sophus::SE3d(),
      bool use_external_guess = false);


  /**
   * @brief Adds points to the map
   * @param points The points to add to the map
   */
  void addToMap(const std::vector<Eigen::Vector3d> &points);


  /**
   * @brief Removes points that are far from a given location
   * @param cloud The point cloud to filter
   * @param point The reference point
   * @return The filtered point cloud
   */
  std::vector<Eigen::Vector3d> removePointsFarFromLocation(const std::vector<Eigen::Vector3d> &cloud,
                                                            const Eigen::Vector3d &point);
  /**
   * @brief Gets the map points
   * @return Vector of points in the map
   */
  std::vector<Eigen::Vector3d> getMap(){
    return voxel_map_.cloud();
  }

  /**
   * @brief Removes points from a cloud that are beyond the maximum distance from origin
   * 
   * This method filters out points from the input cloud that have a Euclidean distance
   * from the origin (0,0,0) greater than the maximum distance threshold. Points are
   * kept if their norm (distance from origin) is less than max_distance_.
   * 
   * @param cloud The input point cloud to filter
   * @return A filtered point cloud containing only points within the maximum distance
   */

std::vector<Eigen::Vector3d> removeFarPoints(std::vector<Eigen::Vector3d> &cloud){
  std::vector<Eigen::Vector3d> pruned_cloud;
  pruned_cloud.reserve(cloud.size());
  std::for_each(cloud.begin(),cloud.end(),
  [&](const auto point){
    if(point.norm()<max_distance_){pruned_cloud.push_back(point);}});
  return pruned_cloud;
  }

 inline bool mapEmpty(){
    return voxel_map_.empty();
  }

  /**
   * @brief Gets the current position
   * @return The current position
   */
  Sophus::SE3d position() const;

  /**
   * @brief Updates the current position
   * @param transformation_matrix The new position
   */
  void updatePosition(const Sophus::SE3d transformation_matrix);

private:
  Registration registration_;
  Sophus::SE3d current_position_;
  Sophus::SE3d pose_diff_;
  AdaptiveThreshold threshold;
  double voxel_factor_;
  double max_distance_;
  double voxel_resolution_alpha_;
  double voxel_resolution_beta_;
  bool imu_integration_enabled_;
  int max_points_per_voxel_;
  bool odom_voxel_downsample_;
  VoxelMap voxel_map_;
  int lfu_prune_counter_; // Counter to track when to prune via LFU
  int lfu_prune_interval_; // Interval for LFU pruning
};

} // namespace cloud
