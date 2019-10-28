/*@
 *@file talker.cpp
 *@author Umang Rastogi
 *@brief Tutotial to demonstrate simple sending of messages over the ROS system
 *@copyright 3-clause BSD
 */

#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
  /*
   * Initialize the talking node
   *
   * init must be called before any other part of the ROS system
   */
  ros::init(argc, argv, "talker");

  /*
   * Initialize the main access point to communications with the ROS system
   */
  ros::NodeHandle n;

  /*
   * Inititlaize the publisher with a topic name and buffer size of messages
   * 
   * Make sure the listener is subscribed to the same topic name
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("UMD", 1000);

  ros::Rate loop_rate(100);

  /*
   * Initialize a variable to count the no. of published messages
   */
  int count = 0;

  /*
   * Start a loop that ends via the Ctrl-C command from keyboard
   */
  while (ros::ok()) {
    /*
     * Initialize a message object to stuff data and publish it
     */
    std_msgs::String message;

    /*
     * Construct the message to be published
     */
    message.data = "Go Terps! " + std::to_string(count);

    /*
     * Add message data to the ROS logger
     */
    ROS_INFO("%s", message.data.data());

    /*
     * Send the message to the subscribed topic
     */
    chatter_pub.publish(message);

    /*
     * Add to handle callbacks
     */
    ros::spinOnce();

    /*
     * Make the system sleep to maintain loop rate
     */
    loop_rate.sleep();

    /*
     * Increment the counter
     */
    ++count;
  }


  return 0;
}
