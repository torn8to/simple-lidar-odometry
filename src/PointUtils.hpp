#pragma once
#include <algorithm>
#include <iterator>
#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace cloud {
    /*
   * @param points The points to transform
   * @param transform The SE3 transformation to apply
   * @return The transformed points
   */
  inline std::vector<Eigen::Vector3d> transformPoints(
      const std::vector<Eigen::Vector3d>& points, 
      const Sophus::SE3d& transform) {
    std::vector<Eigen::Vector3d> transformed_points;
    transformed_points.reserve(points.size());
    std::transform(points.begin(), points.end(), std::back_inserter(transformed_points),
                   [&transform](const Eigen::Vector3d& point) {
                       return transform * point;}
                       );
    return transformed_points;
  }

}
