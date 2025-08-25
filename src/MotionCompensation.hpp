#include <cassert>
#include <vector>
#include <algorithm>
#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

namespace cloud{
/**
 * @brief Performs motion compensation on a point cloud using timestamps and transformation
 * 
 * This function compensates for motion during the LiDAR scan by applying a transformation
 * to each point based on its timestamp relative to the scan start time. The transformation
 * is interpolated using the time difference between the current point and the first point.
 * 
 * @param cloud The input point cloud to be motion compensated
 * @param cloud_timestamps Vector of timestamps corresponding to each point in the cloud
 * @param diff_transformation The differential transformation to apply (typically velocity * dt)
 * @return std::vector<Eigen::Vector3d> The motion-compensated point cloud
 * @note The cloud and timestamps vectors must have the same size
 * @note The diff_transformation should represent the motion between consecutive timestamps
 */

inline std::vector<Eigen::Vector3d> motionDeSkew(const std::vector<Eigen::Vector3d> &cloud,
                                         std::vector<double> &cloud_timestamps,
                                         const Sophus::SE3d &relative_motion){
  assert(cloud.size() == cloud_timestamps.size() && "Cloud and timestamps must have the same size");
  std::vector<Eigen::Vector3d> compensated_cloud;
  compensated_cloud.reserve(cloud.size());
  auto cloud_it = cloud.begin();
  auto timestamps_it = cloud_timestamps.begin();
  double begin_time = cloud_timestamps.front();
  double last_time = cloud_timestamps.back();
  const auto omega = relative_motion.log();

  tbb::parallel_for( 
    tbb::blocked_range<size_t>{0, compensated_cloud.size()},
      [&]( const tbb::blocked_range<size_t> &r){
      for(size_t idx = r.begin(); idx <r.end(); ++idx){
        const Eigen::Vector3d &point = cloud.at(idx);
        const auto  norm_dt = (cloud_timestamps.at(idx)) - begin_time/(last_time- begin_time);
        compensated_cloud.at(idx) = Sophus::SE3d::exp(omega*norm_dt) *point;
      };});
      return compensated_cloud;
    }

} // namespace cloud
