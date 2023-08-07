/* 
 * scout_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MESSENGER_HPP
#define SCOUT_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>

class ScoutROSMessenger
{
public:
    explicit ScoutROSMessenger(ros::NodeHandle *nh);

    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;
    bool pub_tf;

    void SetupSubscription();
    void GetCurrentMotionCmdForSim(double &linear, double &angular);
    void PublishSimStateToROS(double linear, double angular);
    void PublishOdometryToROS(double linear, double angular, double dt);

private:
    ros::NodeHandle *nh_;

    std::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;

    ros::Publisher odom_publisher_;
    ros::Subscriber motion_cmd_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    ros::Time last_time_;
    ros::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
};

#endif /* SCOUT_MESSENGER_HPP */
