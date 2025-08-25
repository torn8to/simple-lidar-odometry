#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

/**
 * shamelessly ripped from kiss_icp 
 * https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Threshold.cpp 
 */
struct AdaptiveThreshold {
    explicit AdaptiveThreshold(double initial_threshold,
                               double min_motion_threshold,
                               double max_range): 
                               min_motion_threshold_(min_motion_threshold),
                               max_range_(max_range),
                               model_sse_(initial_threshold * initial_threshold),
                               num_samples_(1){}


    /// Returns the KISS-ICP adaptive threshold used in registration
    inline void updateModelDeviation(const Sophus::SE3d &current_deviation) {
      const double model_error = [&]() {
      const double theta = Eigen::AngleAxisd(current_deviation.rotationMatrix()).angle();
      const double delta_rot = 2.0 * max_range_ * std::sin(theta / 2.0);
      const double delta_trans = current_deviation.translation().norm();
      return delta_trans + delta_rot;
      }();
        if (model_error > min_motion_threshold_) {
        model_sse_ += model_error * model_error;
        num_samples_++;
      }
    }

    inline double computeThreshold() const { return std::sqrt(model_sse_ / num_samples_); }

    // configurable parameters
    double min_motion_threshold_;
    double max_range_;

    // Local cache for computation
    double model_sse_;
    int num_samples_;
};