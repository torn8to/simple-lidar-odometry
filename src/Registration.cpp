#include <vector>
#include <tuple>

#include"Registration.hpp"
#include "VoxelMap.hpp"
#include "PointToVoxel.hpp"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

namespace Eigen{
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix36d = Eigen::Matrix<double, 3, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
}

using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

namespace{

inline double square(double x) { return x * x; }

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const cloud::VoxelMap &voxel_map,
                                const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
              std::for_each(r.begin(), r.end(), [&](const auto &point) {
                std::tuple<Eigen::Vector3d, double> result = voxel_map.firstNearestNeighborQuery(point);
                const Eigen::Vector3d& closest_neighbor = std::get<0>(result);
                const double distance = std::get<1>(result);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(std::make_pair(point, closest_neighbor));
                }
            });
        });
    return correspondences;
}



LinearSystem BuildLinearSystem(const Correspondences &correspondences,
                               const double kernel_scale) {
    auto compute_jacobian_and_residual = [](const auto &correspondence) {
        const Eigen::Vector3d& source = correspondence.first;
        const Eigen::Vector3d& target = correspondence.second;
        const Eigen::Vector3d residual = source - target;
        Eigen::Matrix36d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
        return std::make_tuple(J_r, residual);
    };

    auto sum_linear_systems = [](LinearSystem a, const LinearSystem &b) {
        a.first = a.first + b.first;
        a.second = a.second + b.second;
        return a;
    };

    auto GM_weight = [&](const double &residual2) {
        return square(kernel_scale) / square(kernel_scale + residual2);
    };

    using correspondence_iterator = Correspondences::const_iterator;
    LinearSystem result = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend()},
        // Identity
        LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator> &r, LinearSystem J) -> LinearSystem {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &correspondence) {
                    std::tuple<Eigen::Matrix36d, Eigen::Vector3d> jr_result = compute_jacobian_and_residual(correspondence);
                    const Eigen::Matrix36d& J_r = std::get<0>(jr_result);
                    const Eigen::Vector3d& residual = std::get<1>(jr_result);
                    const double w = GM_weight(residual.squaredNorm());
                    return LinearSystem(J_r.transpose() * w * J_r,
                                        J_r.transpose() * w * residual);
                });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);

    return result;
  }
}  // namespace


Registration::Registration(int num_iterations, double convergence, int num_threads ):
          max_num_iterations_(num_iterations),
          convergence_(convergence),
          num_threads_(num_threads > 0 ? num_threads : tbb::this_task_arena::max_concurrency()){
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(num_threads_));
  }


Sophus::SE3d Registration::alignPointsToMap(const std::vector<Eigen::Vector3d> &points,
                                            const cloud::VoxelMap &voxel_map,
                                            const Sophus::SE3d &initial_guess,
                                            const double max_distance,
                                            const double kernel_scale){
  std::vector<Eigen::Vector3d> source = points;
  TransformPoints(initial_guess, source);
  int num_iterations;
  Sophus::SE3d T_icp = Sophus::SE3d();
  for (int j = 0; j < max_num_iterations_; ++j) {
    const auto correspondences = DataAssociation(source, voxel_map, max_distance);
    LinearSystem linearSystem = BuildLinearSystem(correspondences, kernel_scale);
    const Eigen::Matrix6d& JTJ = linearSystem.first;
    const Eigen::Vector6d& JTr = linearSystem.second;
    const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
    const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
    TransformPoints(estimation, source);
    T_icp = estimation * T_icp;
    num_iterations = j;
    if (dx.norm() < convergence_) break;
  }
  return T_icp * initial_guess;
}
