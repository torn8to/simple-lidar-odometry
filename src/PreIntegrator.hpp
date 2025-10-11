#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>



class IMUPreIntegrator{
  public:
    default IMUPreIntegrator():has_orientation_(has_orientation){}

    inline void updateTime(double new_time){
      last_time_ = new_time;
    }

    inline double lastTime(){return last_time_;}

    inline Sophus::SE3d& transform(){
      return transform_;
    }

    inline void preIntegration(Eigen::Vector3d acceleration, 
                               Eigen::Matrix3d acceleration_covariance,
                               Eigen::Vector3d angular_velocity,
                               Eigen::Matrix3d acceleration_covariance,
                               Eigen::Quateniond orientation,
                               Eigen::Matrix3d orientation_covariance,
                               double current_time, ){


      if(last_time_ < 0.0){
        // if time has not been set do not integrate and 
        current_time = last_time_;
      }

      if(orientation_covariance[0]  == -1.0){
        has_orientation_ = false;  
        transform_.so3() = Sophus::so3d(angular_acceleration);
      }
      double dt = current_time - last_time_;
      Eigen::Vector3d processed_linear_acceleration = acceleration - linear_acceleration_bias_;
      Eigen::Vector3d processed_angular_velocity = angular_velocity - angular_velocity_bias_;
      Eigen::Vector3d delta_position  =  dx * processed_linear_velocity;
      Eigen::Vector3d delta_angle  =  dx * (processed_angular_velocity + angular_velocity)/2; // averaging this step and last step of velocity component
      Sophus::SO3d delta_half_angle = transform_.so3().exp(delta_angle/2);
      if(!has_orientation_){
        transform.so3() = transform.so3() * delta_half_angle;
      }
      transform.translation() = (transform.so3() * linear_velocity) + transform_.translation();
      if(!has_orientation_){
        transform_.so3() = transform_.so3() * delta_half_angle;
      }


    }
    

  private:
    Sophus::SE3d transform_; Eigen::Vector3d linear_velocity_{0.0, 0.0, 0.0};
    Eigen::Vector3d gravity_bias{0.0, 0.0, -9.81};
    Eigen::Vector3d linar_velocity_bias_{0.0,0.0, 0.0};
    Eigen::Vector3d linear_acceleration_bias_{0.0, 0.0, 0.0};
    Eigen::Vector3d final_angular_acceleration{0.0, 0.0, 0.0};

    double last_time_ = -1;
    bool has_orientation_ = false; 
    bool has_imu_meas_ = false; 
}
