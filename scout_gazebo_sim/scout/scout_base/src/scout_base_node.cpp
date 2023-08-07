#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "scout_base/scout_messenger.hpp"

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  ScoutROSMessenger messenger(&node);

  private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_, std::string("odom"));
  private_node.param<bool>("pub_tf", messenger.pub_tf, true);


  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (ros::ok()) {
      double linear, angular;

      messenger.GetCurrentMotionCmdForSim(linear, angular);
      messenger.PublishSimStateToROS(linear, angular);

      ros::spinOnce();
      rate.sleep();
  }

  return 0;
}
