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
 * @file modifyMessageTest.cpp
 * @author Umang Rastogi
 * @brief Tests various fucntionalities of the modify message service
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
    nh.serviceClient<beginner_tutorials::modifyMessage>("modifyMessageService");
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
    nh.serviceClient<beginner_tutorials::modifyMessage>("modifyMessageService");
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
