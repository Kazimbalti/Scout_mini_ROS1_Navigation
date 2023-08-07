#include <ros/ros.h>
#include <ros/console.h>
#include "odom_converter/odom_converter.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_converter");

    // parse arguments
    ROS_ASSERT(argc>8);
    double x, y, z, yaw;

    if(std::string(argv[1])=="-x")
        x = atof(argv[2]);
    if(std::string(argv[3])=="-y")
        y = atof(argv[4]);
    if(std::string(argv[5])=="-z")
        z = atof(argv[6]);
    if(std::string(argv[7])=="-yaw")
        yaw = atof(argv[8]);
    
    ros::NodeHandle node(""), private_node("~");

    OdomConverter converter(&node, x, y, z, yaw);

    private_node.param<std::string>("odom_frame", converter.odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", converter.base_frame_, std::string("base_link"));
    private_node.param<std::string>("odom_publish_topic_name", converter.odom_publish_topic_name_, std::string("odom"));
    private_node.param<std::string>("odom_gazebo_topic_name", converter.odom_gazebo_topic_name_, std::string("odom_gazebo"));
    private_node.param<bool>("pub_tf", converter.pub_tf, true);
    private_node.param<bool>("add_noise", converter.add_noise, true);

    converter.SetupSubscription();
    
    ros::Rate rate(50);
    while(ros::ok()){
        converter.PublishOdometry();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}