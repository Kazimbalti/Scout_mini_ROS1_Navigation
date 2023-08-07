/*
 * scout_messenger.cpp
 *
 * Created on: Apr 26, 2019 22:14
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_base/scout_messenger.hpp"

#include <tf/transform_broadcaster.h>

ScoutROSMessenger::ScoutROSMessenger(ros::NodeHandle *nh)
    : nh_(nh) {}

void ScoutROSMessenger::SetupSubscription()
{
// odometry publisher
odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);

// cmd subscriber
motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
    "/cmd_vel", 5, &ScoutROSMessenger::TwistCmdCallback, this);
}

void ScoutROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = *msg.get();
// ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

void ScoutROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                double &angular)
{
std::lock_guard<std::mutex> guard(twist_mutex_);
linear = current_twist_.linear.x;
angular = current_twist_.angular.z;
}

void ScoutROSMessenger::PublishSimStateToROS(double linear, double angular)
{
    current_time_ = ros::Time::now();

    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    // publish odometry and tf
    PublishOdometryToROS(linear, angular, dt);

    // record time for next integration
    last_time_ = current_time_;
}

void ScoutROSMessenger::PublishOdometryToROS(double linear, double angular, double dt)
{
    static bool init_run = true;
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }
    // perform numerical integration to get an estimation of pose
    linear_speed_ = linear;
    angular_speed_ = angular;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x+0.01;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    if(pub_tf){
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = current_time_;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;

        tf_msg.transform.translation.x = position_x_;
        tf_msg.transform.translation.y = position_y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = odom_quat;

        tf_broadcaster_.sendTransform(tf_msg);
    }

    // publish odometry and tf messages
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    odom_publisher_.publish(odom_msg);
}
