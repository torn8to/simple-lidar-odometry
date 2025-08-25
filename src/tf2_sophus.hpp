#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Geometry>

namespace tf2 {
    Sophus::SE3d transformToSophus(const geometry_msgs::msg::TransformStamped &transform) {
        Eigen::Vector3d trans = Eigen::Vector3d(transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z);

        Eigen::Quaterniond rot = Eigen::Quaterniond(transform.transform.rotation.w,
                                                    transform.transform.rotation.x,
                                                    transform.transform.rotation.y,
                                                    transform.transform.rotation.z);
        return Sophus::SE3d(Sophus::SO3d(rot), trans);
    }


    geometry_msgs::msg::TransformStamped sophusToTransform(const Sophus::SE3d &pose) { 
        const Eigen::Quaterniond& quaternion = pose.so3().unit_quaternion();
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.transform.translation.x = pose.translation().x();
        transform_msg.transform.translation.y = pose.translation().y();
        transform_msg.transform.translation.z = pose.translation().z();
        transform_msg.transform.rotation.x = quaternion.x();
        transform_msg.transform.rotation.y = quaternion.y();
        transform_msg.transform.rotation.z = quaternion.z();
        transform_msg.transform.rotation.w = quaternion.w();
        return transform_msg;
    }
}