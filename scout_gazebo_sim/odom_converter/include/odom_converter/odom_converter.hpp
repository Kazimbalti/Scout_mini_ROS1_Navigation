#ifndef ODOM_CONVERTER_HPP
#define ODOM_CONVERTER_HPP

#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class OdomConverter{
    public:
        OdomConverter(ros::NodeHandle *nh, double initial_x_, double initial_y_, double initial_z_, double initial_yaw_);

        std::string odom_frame_;
        std::string base_frame_;
        std::string odom_publish_topic_name_;
        std::string odom_gazebo_topic_name_;
        bool pub_tf;
        bool add_noise;

        void SetupSubscription();
        void PublishOdometry();
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    private:
        ros::NodeHandle *nh_;
        
        std::mutex odom_mutex_;

        ros::Publisher odom_publisher_;
        ros::Subscriber odom_subscriber_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        bool odom_init_;
        nav_msgs::Odometry odom_groudtruth_;

        double initial_x_;
        double initial_y_;
        double initial_z_;
        double initial_yaw_;
        tf2::Quaternion initial_quat_;

        double linear_covariance_[2];
        double angular_covariance_;
        double linear_error_accumulate_[2] = {0};
        double angular_error_accumulate_ = 0;
        
};

#endif