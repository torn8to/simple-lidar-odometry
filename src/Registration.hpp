#pragma once

#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "VoxelMap.hpp"

class Registration{
  public:
    explicit Registration(int num_iterations, double convergence, int num_threads);

    Sophus::SE3d alignPointsToMap(const std::vector<Eigen::Vector3d> &points, 
                                  const cloud::VoxelMap &voxel_map,
                                  const Sophus::SE3d &initial_guess,
                                  const double max_coorespondence_distance,
                                  const double kernel_scale);
      
  private:  
    int max_num_iterations_;
    double convergence_;
    int num_threads_;
};
