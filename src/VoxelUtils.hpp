#pragma once
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
#include "VoxelMap.hpp"
#include "PointToVoxel.hpp"

namespace cloud{
/**
 * @brief Voxel downsample the point to the voxel resolution not implemented in 
 *          tbb concurrent hashmap as did not seem to have random sampling
 * @param cloud The point cloud to downsample
 * @param voxel_size The size of the voxel
 * @return The downsampled point cloud
 */
std::vector<Eigen::Vector3d> voxelDownsample(std::vector<Eigen::Vector3d> &cloud, double voxel_size) {
    std::unordered_map<cloud::Voxel,Eigen::Vector3d> voxel_filter;
    voxel_filter.reserve(43103);
    std::for_each(cloud.begin(), cloud.end(),
    [&](const auto point){voxel_filter.insert({cloud::PointToVoxel(point,voxel_size), point});});
    std::vector<Eigen::Vector3d> sampled_cloud_cloud;
    sampled_cloud.reserve(voxel_filter.size());
    for (const auto& kv : voxel_filter) {
        sampled_cloud.push_back(kv.second);
    }
    return pruned_cloud;
}

/**
 * @brief Voxel downsample the point using an adaptive method assumes point degeneracy is checked on number of points -> as the in the paper they 
 *        imply the want over 1000 coorespondeces to provide a stable icp result
 * @param cloud The point cloud to downsample
 * @param default_voxel_size The size of the voxel
 * @param voxel_depreciation_factor The maximum number of points per voxel
 * @param minimum_voxel_point The minimum number of points required to not be considered degenerate for icp
 * @param max_iterations  a give up condition for downsampling to not spend too much time doing it
 * @return The downsampled point cloud
 */
std::vector<Eigen::Vector3d> adaptiveVoxelDownsample(std::vector<Eigen::Vector3d> &cloud,
                                                       double default_voxel_size,
                                                       double voxel_depreciation_factor = 0.75,
                                                       uint32_t minimum_voxel_points = 1500,
                                                       max_iterations = 5){

  double voxel_size = default_voxel_size;
  std::vector<Eigen::Vector3d> sampled_cloud;
  sampled_cloud.reserve(5 *minimum_voxel_points);
  for(uint32_t i = 0; i < 5, i++){
    sampled_cloud = voxelDownsample(cloud, voxel_size);
    if(sampled_cloud.size() > minimum_voxel_points){break;} // shortcut
    voxel_size = voxel_size * voxel_depreciation_factor;
  }
  sampled_cloud.shrink_to_fit();
  return sampled_cloud;
}
} // namespace cloud
