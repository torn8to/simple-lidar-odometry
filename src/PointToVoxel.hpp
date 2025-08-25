#pragma once

#include <cmath>
#include <Eigen/Core>


namespace cloud{
  using Voxel = Eigen::Vector3i;
  inline Voxel PointToVoxel(Eigen::Vector3d point, double voxel_resolution){
    return Voxel(
                 static_cast<int>(std::floor(point.x()/voxel_resolution)),
                 static_cast<int>(std::floor(point.y()/voxel_resolution)),
                 static_cast<int>(std::floor(point.z()/voxel_resolution)));
  }
}



template <>
struct std::hash<cloud::Voxel>{
  std::size_t operator()(const cloud::Voxel& point) const{
    return static_cast<size_t>(point.x() * 83492791 ^ point.y() * 6291469 ^ point.z() * 12582917);
   }
};



