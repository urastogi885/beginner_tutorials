/**
 * MIT License
 * 
 * @copyright (c) 2020 Umang Rastogi
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE
 * 
 * @file talker.cpp
 * @author Umang Rastogi
 * @brief Tutotial to demonstrate simple receipt of messages over the ROS system
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/*
 * Callback to handle messsage received on the topic
 *
 * Update ROS logger with info from the listener
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard: " << msg->data.data());
}


int main(int argc, char **argv) {
  /// Initialize the talking node
  /// init must be called before any other part of the ROS system
  ros::init(argc, argv, "listener");
  /// Initialize the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Inititlaize the publisher with a topic name and buffer size of messages
  /// Make sure you have subscribed to the correct topic
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
  /// Add to handle callbacks
  ros::spin();

  return 0;
}
