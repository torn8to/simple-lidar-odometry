#pragma once
#include <memory>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


class FasterBroadcaster : public rclcpp::Node{
    public:
    FasterBroadcaster():Node("FasterBroadcaster"){
        this->declare_parameter("hz_transform_rate", 50);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        msg_ = std::make_shared<geometry_msgs::msg::TransformStamped>();

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
            std::bind(&FasterBroadcaster::OdometryCallback, this, std::placeholders::_1));

        int hz = this->get_parameter("hz_transform_rate").as_int();
        auto period = std::chrono::milliseconds(static_cast<int>(1001/hz));

        timer_ = this->create_wall_timer(period,std::bind(&FasterBroadcaster::TimerCallback, this));\

        auto parameter_reaction_callback = 
            [this](const std::vector<rclcpp::Parameter>& params){
                for(const auto & param : params){
                    if(param.get_name() == "hz_transform_rate"){
                        int hz = param.get_value<int>();
                        auto new_period = std::chrono::milliseconds(static_cast<int>(1001/hz));
                        timer_ = this->create_wall_timer(new_period,
                                std::bind(&FasterBroadcaster::TimerCallback, this));
                    }
                }
            };

        post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(parameter_reaction_callback);
    }


    void TimerCallback(){
        if(message_received_){
            msg_->header.stamp = this->get_clock()->now();
            tf_broadcaster_->sendTransform(*msg_);
        }
    }
        
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
        message_received_ = true;
        msg_->header.frame_id = odom_msg->header.frame_id;
        msg_->child_frame_id = odom_msg->child_frame_id;
        msg_->transform.translation.x = odom_msg->pose.pose.position.x;
        msg_->transform.translation.y = odom_msg->pose.pose.position.y;
        msg_->transform.translation.z = odom_msg->pose.pose.position.z;
        msg_->transform.rotation = odom_msg->pose.pose.orientation;
    }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_parameters_callback_handle_; 

        geometry_msgs::msg::TransformStamped::SharedPtr msg_;
        bool message_received_;
};