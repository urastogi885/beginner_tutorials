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
 * @brief Tutotial to demonstrate simple receipt of messages over the ROS system
 */

/// Add standard libraries
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_msgs/String.h>
/// Add custom libraries
#include "beginner_tutorials/modifyMessage.h"

/**
 * @brief To test the existence of service
 * @param TestSuite, testCase
 * @return void
 */
TEST(ModifyMessageTest, checkServiceExistence) {
  /// Initialize the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Initialize the service-client object
  ros::ServiceClient client = 
    nh.serviceClient<beginner_tutorials::modifyMessage>("modifyMessage_service");
  /// Check the existence of service
  bool serviceExists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(serviceExists);
}

/**
 * @brief To test the modification of message
 * @param TestSuite, testCase
 * @return void
 */
TEST(ModifyMessageTest, checkModifyMessage) {
  /// Initialize the main access point to communications with the ROS system
  ros::NodeHandle nh;
  /// Initialize the service-client object
  ros::ServiceClient client = 
    nh.serviceClient<beginner_tutorials::modifyMessage>("modifyMessage_service");
  /// Initialize the service object
  beginner_tutorials::modifyMessage modify;
  /// requesting the new string to service
  modify.request.number = 10;
  client.call(modify);
  /// comparing the service response with the requested string
  EXPECT_STREQ("Go Terps! Go Terps!", modify.response.changedMessage.c_str());
}

/**
 * @brief main
 * @return int
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "publisher");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}