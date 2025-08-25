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
 * @brief Voxel downsample the point to the voxel resolution
 * @param cloud The point cloud to downsample
 * @param voxel_size The size of the voxel
 * @param max_points_per_voxel The maximum number of points per voxel
 * @return The downsampled point cloud
 */
std::vector<Eigen::Vector3d> voxelDownsample(std::vector<Eigen::Vector3d> &cloud, double voxel_size) {
    std::unordered_map<cloud::Voxel,Eigen::Vector3d> voxel_filter;
    voxel_filter.reserve(43103);
    std::for_each(cloud.begin(), cloud.end(),
    [&](const auto point){voxel_filter.insert({cloud::PointToVoxel(point,voxel_size), point});});
    std::vector<Eigen::Vector3d> pruned_cloud;
    pruned_cloud.reserve(voxel_filter.size());
    for (const auto& kv : voxel_filter) {
        pruned_cloud.push_back(kv.second);
    }
    return pruned_cloud;
}
} // namespace cloud