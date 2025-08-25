#include "VoxelMap.hpp"
#include <algorithm>
#include <tuple>

namespace {
using cloud::Voxel;
  

static const std::array<Voxel, 27> voxel_shifts{
    {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
     Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
     Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
     Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
     Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
     Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};
} 

namespace cloud{
  void VoxelMap::addPoints(const std::vector<Eigen::Vector3d> &points){
    double resolution_spacing = std::sqrt(voxel_resolution_*voxel_resolution_/max_points_per_voxel_);
    for(const auto &point : points){
      const Voxel voxel = PointToVoxel(point, voxel_resolution_);
      auto query = map_.find(voxel);
      if(query == map_.end()){
        std::vector<Eigen::Vector3d> vec;
        vec.reserve(max_points_per_voxel_);
        vec.push_back(point);
        map_.insert({voxel,std::move(vec)});
      }else{
        auto &voxel_data = query->second;
        if(voxel_data.size() == max_points_per_voxel_ || 
         std::any_of(voxel_data.begin(), voxel_data.end(),
         [&](const auto &existing_point){return (existing_point - point).norm()
          < resolution_spacing;})){
          continue;
        }else{
          voxel_data.emplace_back(point);
        }
      }
    }
  }
  
  std::vector<Eigen::Vector3d> VoxelMap::transform_cloud( 
        const std::vector<Eigen::Vector3d> &points, 
        const Sophus::SE3d &transform){
    std::vector<Eigen::Vector3d>  transformed_points;
    transformed_points.reserve(points.size());
    for(const Eigen::Vector3d &point: points){
      transformed_points.emplace_back(transform * point);
    }
   return transformed_points;
  }


  std::vector<Eigen::Vector3d> VoxelMap::removeFarPoints(const std::vector<Eigen::Vector3d> &cloud){
    double max_range_squared = max_range_ * max_range_;
    std::vector<Eigen::Vector3d> pruned_cloud;
    pruned_cloud.reserve(cloud.size());
    std::for_each(cloud.begin(), cloud.end(),[&](const auto point){
                    if(point.squaredNorm() < max_range_squared){pruned_cloud.push_back(point);}
                  });
    return pruned_cloud;
  }

std::vector<Eigen::Vector3d> VoxelMap::cloud() const {
    std::vector<Eigen::Vector3d> cloud;
    cloud.reserve(map_.size() * static_cast<size_t>(max_points_per_voxel_));
    std::for_each(map_.begin(), map_.end(),[&](const auto map_element){
                  const std::vector<Eigen::Vector3d> voxel_data = map_element.second;
                  cloud.insert(cloud.end(), voxel_data.begin(), voxel_data.end());
                  });
    return cloud;
  }

std::tuple<Eigen::Vector3d, double> VoxelMap::firstNearestNeighborQuery(const Eigen::Vector3d &point) const {
  const cloud::Voxel voxel = cloud::PointToVoxel(point, voxel_resolution_);
  Eigen::Vector3d closest_neighbor =  Eigen::Vector3d::Zero();
  double min_distance = std::numeric_limits<double>::max();
  std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(),
                [&](const auto voxel_shift){
                    const auto query = map_.find(voxel+voxel_shift);
                    if(query != map_.end()){
                       const std::vector<Eigen::Vector3d> voxel_points = query->second;
                       const Eigen::Vector3d &neighbor = *std::min_element(voxel_points.begin(), voxel_points.end(), 
                                [&](const auto &lhs, const auto &rhs){
                                  return (lhs - point).norm() < (rhs - point).norm();
                                });
                        double distance = (neighbor - point).norm();
                        if (distance < min_distance){
                          closest_neighbor = neighbor;
                          min_distance = distance;
                        }
                      }
                });
   return std::tuple<Eigen::Vector3d, double>(closest_neighbor, min_distance);
}


void VoxelMap::removePointsFarFromOrigin(const Eigen::Vector3d& origin){
  const auto max_distance_sq  = max_range_ * max_range_;
  for(auto it = map_.begin(); it != map_.end();){
    const auto &[voxel, voxel_points] = *it;
    const Eigen::Vector3d &f_point = voxel_points.front();
    if((origin-f_point).squaredNorm() > max_distance_sq){
      it = map_.erase(it);
    }
    else{
      ++it;
    }
  }
}


void VoxelMap::lfuUpdate(const cloud::Voxel &voxel){
  lfu_cache_.put(voxel);
}

int VoxelMap::lfuGet(const cloud::Voxel &voxel){
  return lfu_cache_.get(voxel);
}

void VoxelMap::pruneViaLfu(){
  for(auto it = map_.begin(); it != map_.end(); ++it){
    auto voxel = it->first;
    if(!(lfuGet(voxel) > 0)){
      map_.erase(voxel);
    }
  }
}


}  // namespace cloud

