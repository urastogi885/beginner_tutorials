/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2019, Umang Rastogi
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file talker.cpp
 * @author Umang Rastogi
 * @brief Tutotial to demonstrate simple sending of messages over the ROS system
 */

/// Add standard libraries
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
/// Add custom libraries
#include "beginner_tutorials/messageManipulator.h"

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
bool modifyMessage(beginner_tutorials::messageManipulator::Request &req,
                beginner_tutorials::messageManipulator::Response &res);

int main(int argc, char **argv) {
  /// Initialize the talking node
  /// init must be called before any other part of the ROS system
  ros::init(argc, argv, "talker");
  /// Initialize the main access point to communications with the ROS system
  ros::NodeHandle node;
  /// Set default frequency
  int freq = 100;

  /// Check if user has entered frequency
  if (argc == 2) {
    if (atoi(argv[1]) < 0) {
      /// Report fatal error for negative frequency
      ROS_FATAL("Invalid rate");
      return 1;
    } else if (atoi(argv[1]) > 1000) {
      /// set error log if rate is too high
      ROS_ERROR("Rate too large");
    }
    freq = atoi(argv[1]);
  } else {
    /// if no argument is found, exit the program
    ROS_FATAL("No argument passed for frequency");
    return 1;
  }

  /// Inititlaize the publisher with a topic name and buffer size of messages
  /// Make sure the listener is subscribed to the same topic name
  ros::Publisher chatter_pub = node.advertise<std_msgs::String>("UMD", 1000);
  /// Initialize the server with a service name and the manipulator function
  ros::ServiceServer server = node.advertiseService("manipulate_service", \
                          modifyMessage);
  /// Set loop frequency
  ros::Rate loop_rate(freq);
  /// Update the modified message before
  rosMessage.modifiedMessage = rosMessage.baseMessage;
  /// Initialize a variable to count the no. of published messages
  int count = 0;

  /// Start a loop that ends via the Ctrl-C command from keyboard
  while (ros::ok()) {
    /// Initialize a message object to stuff data and publish it
    std_msgs::String message;
    /// Construct the message to be published
    message.data = rosMessage.modifiedMessage + " [" + \
              std::to_string(count) + "]";
    /// Add message data to the ROS logger
    ROS_INFO("%s", message.data.data());
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

bool modifyMessage(beginner_tutorials::messageManipulator::Request &req,
                beginner_tutorials::messageManipulator::Response &res) {
  /// Instantiate necessary variables
  int number = req.number;
  /// Add info level message
  ROS_DEBUG("Received number: " << number);

  if (number < 0) {
    /// Report error for negative
    ROS_ERROR("Number not positive");
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

  ROS_WARN("Publishing message modified");
  ROS_DEBUG("Output generated");

  return true;
}
