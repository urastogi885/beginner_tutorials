/*@
 *@file talker.cpp
 *@author Umang Rastogi
 *@brief Tutotial to demonstrate simple receipt of messages over the ROS system
 *@copyright MIT
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/*
 * Callback to handle messsage received on the topic
 *
 * Update ROS logger with info from the listener
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.data());
}


int main(int argc, char **argv) {
  /*
   * Initialize the talking node
   *
   * init must be called before any other part of the ROS system
   */
  ros::init(argc, argv, "listener");

  /*
   * Initialize the main access point to communications with the ROS system
   */  
  ros::NodeHandle n;

  /*
   * Inititlaize the publisher with a topic name and buffer size of messages
   * 
   * Make sure you have subscribed to the correct topic
   */
  ros::Subscriber sub = n.subscribe("UMD", 1000, chatterCallback);

  /*
   * Add to handle callbacks
   */
  ros::spin();

  return 0;
}
