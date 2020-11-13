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
 * @brief Tutotial to demonstrate simple sending of messages over the ROS system
 */

/// Add standard libraries
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
/// Add custom libraries
#include "beginner_tutorials/modifyMessage.h"

/// Define structure for the ROS message
struct RosMessage {
  const std::string baseMessage = "Go Terps!";
  std::string modifiedMessage;
};

/// Initialize object for the ROS message
RosMessage rosMessage;

/**
 * @brief function to modify the original message
 * @param req int number entered by client to modify the message
 * @param res int result
 * @return boolean
 */
bool modifyMessage(beginner_tutorials::modifyMessage::Request &req,
                beginner_tutorials::modifyMessage::Response &res);

int main(int argc, char **argv) {
  /// Initialize the talking node
  /// init must be called before any other part of the ROS system
  ros::init(argc, argv, "talker");
  /// Initialize the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Initialize the transform-broadcaster object
  static tf::TransformBroadcaster tbr;
  /// Initialize the transform object
  tf::Transform transform;
  /// Initialize the quaternion object
  tf::Quaternion quaternion;
  /// Set the default frequency
  int freq = 100;
  /// Set the origin for the transform
  transform.setOrigin(tf::Vector3(1.0, 3.0, 5.0));
  /// Set rotation using roll, pitch, and yaw
  quaternion.setRPY(10, 20, 15);
  transform.setRotation(quaternion);

  /// Check if user has entered frequency
  if (argc == 2) {
    if (atoi(argv[1]) < 0) {
      /// Report fatal error for negative frequency
      ROS_FATAL_STREAM("Invalid rate");
      return 1;
    } else if (atoi(argv[1]) > 1000) {
      /// Set error log if rate is too high
      ROS_ERROR_STREAM("Rate is too large");
    }
    /// Set publishing freqency
    freq = atoi(argv[1]);
  } else {
    /// if no argument is found, exit the program
    ROS_FATAL_STREAM("No argument passed for frequency");
    return 1;
  }

  /// Inititlaize the publisher with a topic name and buffer size of messages
  /// Make sure the listener is subscribed to the same topic name
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  /// Initialize the server with a service name and the manipulator function
  ros::ServiceServer server = nh.advertiseService("modifyMessageService", \
                          modifyMessage);
  /// Set loop frequency
  ros::Rate loop_rate(freq);
  /// Update the modified message before
  rosMessage.modifiedMessage = rosMessage.baseMessage;
  /// Initialize a counter for the no. of published messages
  int count = 0;

  /// Start a loop that ends via the Ctrl-C command from keyboard
  while (ros::ok()) {
    /// Broadcast the transform information
    tbr.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    /// Initialize a message object to stuff data and publish it
    std_msgs::String message;
    /// Construct the message to be published
    message.data = rosMessage.modifiedMessage + " [" + \
              std::to_string(count) + "]";
    /// Add message data to the ROS logger
    ROS_INFO_STREAM(message.data.data());
    /// Send the message to the subscribed topic
    chatter_pub.publish(message);
    /// Add to handle callbacks
    ros::spinOnce();
    /// Make the system sleep to maintain loop rate
    loop_rate.sleep();
    /// Increment the counter
    ++count;
  }

  return 0;
}

bool modifyMessage(beginner_tutorials::modifyMessage::Request &req,
                beginner_tutorials::modifyMessage::Response &res) {
  /// Instantiate necessary variables
  int number = req.number;
  /// Add info level message
  ROS_DEBUG_STREAM("Received number: " << number);

  if (number < 0) {
    /// Report error for negative
    ROS_ERROR_STREAM("Number not positive");
  }

  if (number % 2 == 0) {
    /// Repeat the original message twice
    rosMessage.modifiedMessage = rosMessage.baseMessage + " " + \
                            rosMessage.baseMessage;
  } else {
    /// Repeat the original thrice
    rosMessage.modifiedMessage = rosMessage.baseMessage + " " + \
                            rosMessage.baseMessage + " " + \
                            rosMessage.baseMessage;
  }

  /// Add modified message as the response of the service
  res.changedMessage = rosMessage.modifiedMessage;

  ROS_WARN_STREAM("Publishing message modified");
  ROS_DEBUG_STREAM("Output generated");

  return true;
}
