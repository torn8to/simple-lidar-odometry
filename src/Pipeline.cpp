#include "Pipeline.hpp"
#include "VoxelUtils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace cloud {

Pipeline::Pipeline(const PipelineConfig &config)
  : registration_(config.num_iterations,  config.convergence, config.num_threads),
    current_position_(Sophus::SE3d()),
    voxel_factor_(config.voxel_factor),
    max_distance_(config.max_distance),
    voxel_resolution_alpha_(config.voxel_resolution_alpha),
    voxel_resolution_beta_(config.voxel_resolution_beta),
    max_points_per_voxel_(config.max_points_per_voxel),
    odom_voxel_downsample_(config.odom_downsample),
    voxel_map_((config.max_distance/config.voxel_factor) * config.voxel_resolution_alpha,
    config.max_distance ,
    config.max_points_per_voxel),
    threshold(config.initial_threshold, config.min_motion_threshold, config.max_distance),
    fixed_threshold_(config.fixed_threshold),
    lfu_prune_counter_(0),
    lfu_prune_interval_(config.lfu_prune_interval){}

Pipeline::~Pipeline() {
  // Cleanup if needed
}


std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>> Pipeline::odometryUpdate(std::vector<Eigen::Vector3d> &cloud,
                                           const Sophus::SE3d &external_guess,
                                           bool use_external_guess){
  std::vector<Eigen::Vector3d> cloud_voxel_odom;
  if (odom_voxel_downsample_){
    cloud_voxel_odom = voxelDownsample(cloud, max_distance_ / voxel_factor_ * voxel_resolution_beta_);
  }
  else{
    cloud_voxel_odom = cloud;
  }
  std::vector<Eigen::Vector3d> cloud_voxel_mapping = voxelDownsample(cloud,max_distance_ / voxel_factor_ * voxel_resolution_alpha_);

  Sophus::SE3d initial_guess;
  if (use_external_guess){
    initial_guess = external_guess;
  }else{
    initial_guess = pose_diff_ * position();
  }
  double sigma;

  if (fixed_threshold_ < 0.0){
    sigma = threshold.computeThreshold();// adpative thresholding for kiss icp
   }else{
    sigma = fixed_threshold_;
  }

  std::vector map_cloud = voxel_map_.cloud();
  Sophus::SE3d new_position = registration_.alignPointsToMap(
    cloud_voxel_odom,
    voxel_map_,
    initial_guess,
    3.0 * sigma,
    sigma);

  pose_diff_ = new_position * position().inverse();
  threshold.updateModelDeviation(pose_diff_);

  std::vector<Eigen::Vector3d> cloud_voxel_mapping_transformed = voxel_map_.transform_cloud(cloud_voxel_mapping, new_position);

  voxel_map_.addPoints(cloud_voxel_mapping_transformed);
  voxel_map_.removePointsFarFromOrigin(new_position.translation());
  updatePosition(new_position);
  return std::make_tuple(new_position, cloud_voxel_mapping);
}



/**
 * @brief Adds a set of points to the voxel map.
 *
 * This method inserts the provided points into the internal voxel map structure,
 * updating the map with new data. The points are typically assumed to be in the
 * global or map frame and will be voxelized and managed according to the map's
 * configuration. assumes points have been already trnasformed.
 *
 * @param points The vector of 3D points to add to the map.
 */
void Pipeline::addToMap(const std::vector<Eigen::Vector3d> &points) {
  voxel_map_.addPoints(points);
}


Sophus::SE3d Pipeline::position() const {
  return current_position_;
}

void Pipeline::updatePosition(const Sophus::SE3d transformation_matrix) {
  current_position_ = transformation_matrix;
}

} // namespace cloud
