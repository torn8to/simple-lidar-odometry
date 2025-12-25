#include <vector>
#include <tuple>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <rclcpp/rclcpp.hpp>

/**
 * this class handles imu preIntegration for being used to plug into lidar odomerty
 *
 */
class IMUPreIntegrator{
  public:
    IMUPreIntegrator(bool has_orientation = false,
                     uint imu_update_queue_size = 1000)
                    :has_orientation_(has_orientation){
    imu_update_queue_.reserve(imu_update_queue_size);
  }

    inline void updateTime(double new_time){
      last_time_ = new_time;
    }
    // returns the current_time_ v. last_time
    inline double lastTime(){return last_time_;}

    /**
     *  sets whether or not the 
     */
    inline void setHasOrientation(bool has_orientation){
      has_orientation_ = has_orientation;
    }

    /**
     *  mutator method that passes a reference to the underlying you just set 
     *  the this as lhs of a move semantics statement
     *  integrator.transform = Sophus::SE3d();
     */
    inline Sophus::SE3d& transform(){
      return transform_;
    }

    /**
     *  a getter method that passes the trnasform
     */
    inline Sophus::SE3d const getTransform(){
      return transform_;
    }
    

    inline Eigen::Vector3d& angular_velocity_bias(){
      return angular_velocity_bias_;
    }


    inline Eigen::Vector3d const getAngularVelocityBias(){
      return angular_velocity_bias_;
    }

    /**
    *   This method integrates the imu data it takes in imu msg outputs to be used for pre integration.  Ensure that has_orientation_ has been set to the right state before using
    *   @param acceleration a 3 dimenional vector of acceleration relativ to the imu frame
    * 
    *   @param angular_velocity a 3 dimenional vector of acceleration relativ to the imu frame
    *   @param orientation is used only if ha_orientation is enable
    */
    inline void preIntegrate(Eigen::Vector3d linear_acceleration, 
                               Eigen::Vector3d angular_velocity,
                               Eigen::Quaterniond orientation,
                               double current_time){

      if(!has_imu_meas_){
        last_time_ = current_time;
        has_imu_meas_ = true;
      }

      if(has_orientation_){
        transform_.so3() = Sophus::SO3d(orientation);
      }
      //  angular velocity handling
      double dt = current_time - last_time_;
      Eigen::Vector3d processed_angular_velocity = angular_velocity - angular_velocity_bias_;
      Eigen::Vector3d delta_angle  =  dt * (processed_angular_velocity + angular_velocity)/2;
      Sophus::SO3d delta_half_angle = transform_.so3().exp(delta_angle/2);

      if(!has_orientation_){
        transform_.so3() = transform_.so3() * delta_half_angle;
      }
      // handling acceleration
      Eigen::Vector3d delta_linear_velocity = dt * (linear_acceleration - 
                                                  transform_.so3().inverse() * gravity_bias_
                                                  - linear_acceleration_bias_);

      RCLCPP_INFO(rclcpp::get_logger("odometry"), "the delta_linear_velocity is %a %a %a",
                  delta_linear_velocity.x(),
                  delta_linear_velocity.y(),
                  delta_linear_velocity.z());


      RCLCPP_INFO(rclcpp::get_logger("odometry"), "the linear_velocity is %a %a %a",
                  linear_velocity_.x(),
                  linear_velocity_.y(),
                  linear_velocity_.z());

      transform_.translation() = (transform_.so3() * (delta_linear_velocity/2.0 + linear_velocity_)) 
                                                    + transform_.translation();

      linear_velocity_ += delta_linear_velocity;
      if(!has_orientation_){
        transform_.so3() = transform_.so3() * delta_half_angle;
      }
      last_time_ = current_time;
      imu_update_queue_.emplace_back(std::make_tuple(
            linear_acceleration, angular_velocity, orientation, current_time));
      if(imu_update_queue_.size() == imu_update_queue_.capacity()){
        imu_update_queue_.erase(imu_update_queue_.begin()) ;
      }
    }

    /**
    * resets state to default 
    * has_orientation_ is not reset to false its state is preserved
    */
    inline void reset(){
      transform_  = Sophus::SE3d();
      linear_velocity_ = Eigen::Vector3d::Zero();
      has_imu_meas_ = false;
      imu_update_queue_.clear();
    }

    inline void pruneImuUpdatesBefore(double time){
      std::vector<imu_update> new_queue;
      new_queue.reserve(imu_update_queue_.capacity());
      std::for_each(imu_update_queue_.begin(),
                    imu_update_queue_.end(),
                    [&](const auto update){
                      if(std::get<double>(update) < time){
                        new_queue.push_back(update);                       
                      }
                    });
      imu_update_queue_ = new_queue;
    }
    

  private:
    using imu_update = std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Quaterniond, double>;

    std::vector<imu_update> imu_update_queue_;
    Sophus::SE3d transform_;
    Eigen::Vector3d linear_velocity_{0.0, 0.0, 0.0};
    Eigen::Vector3d gravity_bias_{0.0, 0.0, -9.81};
    Eigen::Vector3d angular_velocity_bias_{0.0,0.0, 0.0};
    Eigen::Vector3d linear_acceleration_bias_{0.0, 0.0, 0.0};

    double last_time_ = -1;
    bool has_orientation_ = false; 
    bool has_imu_meas_ = false; 
};
