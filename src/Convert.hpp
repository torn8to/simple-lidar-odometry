#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <sophus/se3.hpp>
#include <tuple>

namespace cloud {
  std::vector<Eigen::Vector3d> convertMsgToCloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    std::vector<Eigen::Vector3d> cloud;
    cloud.reserve(cloud_msg->height * cloud_msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> x_iter(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iter(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iter(*cloud_msg, "z");

    for(; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
      cloud.push_back(Eigen::Vector3d(
        static_cast<double>(*x_iter),
        static_cast<double>(*y_iter),
        static_cast<double>(*z_iter)
      ));
    }
    return cloud;
  }

  /**
   * @brief Creates an XYZ point cloud message
   * @param points The point cloud data to convert to a message
   * @return PointCloud2 message containing the points
   */
  void convertCloudToMsg(const std::vector<Eigen::Vector3d> &points,
    sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    msg->height = 1;
    msg->width = points.size();
    msg->fields.resize(3);
    msg->point_step = 12; //3*4
    msg->is_bigendian = false;
    msg->row_step = msg->point_step * msg->width;
    msg->is_dense = true;
    msg->data.resize(msg->row_step);

    msg->fields[0].name = "x";
    msg->fields[0].offset = 0;
    msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[0].count = 1;

    msg->fields[1].name = "y";
    msg->fields[1].offset = 4;
    msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[1].count = 1;

    msg->fields[2].name = "z";
    msg->fields[2].offset = 8;
    msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[2].count = 1;

    sensor_msgs::PointCloud2Iterator<float> x_iter(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iter(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iter(*msg, "z");
    for(size_t i = 0; i < points.size(); ++i, ++x_iter, ++y_iter, ++z_iter) {
      *x_iter = static_cast<float>(points[i].x());
      *y_iter = static_cast<float>(points[i].y());
      *z_iter = static_cast<float>(points[i].z());
    }
  }

  std::vector<double> extractTimestampsFromCloudMsg(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    std::vector<double> timestamps;
    timestamps.reserve(cloud_msg->height * cloud_msg->width);
    
    // Check if the "timestamp" field exists in the point cloud message
    bool has_timestamp_field = false;
    for (const auto& field : cloud_msg->fields) {
      if (field.name == "timestamp") {
        has_timestamp_field = true;
        break;
      }
    }
    
    // If timestamp field exists, extract timestamps
    if (has_timestamp_field) {
      try {
        sensor_msgs::PointCloud2ConstIterator<double> time_iter(*cloud_msg, "timestamp");
        for(; time_iter != time_iter.end(); ++time_iter) {
          timestamps.push_back(static_cast<double>(*time_iter));
        }
      } catch (const std::exception& e) {
        // If extraction fails, return empty timestamps
        timestamps.clear();
      }
    }
    
    return timestamps;
}

  /**
   * @brief Extracts odometry data from a nav_msgs::msg::Odometry message
   * @param odom_msg The odometry message to extract data from
   * @return A tuple containing the pose as Sophus::SE3d, linear_velocity, rotational velocity as Eigen::Vector3d, and timestamp as double
   */
  std::tuple<Sophus::SE3d, Eigen::Vector3d, Eigen::Vector3d, double> extractOdometryData(nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    Eigen::Vector3d translation(
      odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z
    );
    
    Eigen::Quaterniond rotation(
      odom_msg->pose.pose.orientation.w,
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z
    );
    
    // Create Sophus::SE3d from translation and rotation
    Sophus::SE3d pose(Sophus::SO3d(rotation), translation);
    
    Eigen::Vector3d linear_velocity(
      odom_msg->twist.twist.linear.x,
      odom_msg->twist.twist.linear.y,
      odom_msg->twist.twist.linear.z
    );


    Eigen::Vector3d angular_velocity(
      odom_msg->twist.twist.angular.x,
      odom_msg->twist.twist.angular.y,
      odom_msg->twist.twist.angular.z
    );
    
    // Extract timestamp
    double timestamp = static_cast<double>(odom_msg->header.stamp.sec) + 
                      static_cast<double>(odom_msg->header.stamp.nanosec) * 1e-9;
    
    return std::make_tuple(pose, linear_velocity, angular_velocity, timestamp);
  }

} // namespace cloud

