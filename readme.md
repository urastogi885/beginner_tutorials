
# ROS Beginner's Tutorial
[![Build Status](https://travis-ci.org/urastogi885/beginner_tutorials.svg?branch=master)](https://travis-ci.org/urastogi885/beginner_tutorials)
[![Coverage Status](https://coveralls.io/repos/github/urastogi885/beginner_tutorials/badge.svg?branch=master)](https://coveralls.io/github/urastogi885/beginner_tutorials?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/urastogi885/beginner_tutorials/blob/master/LICENSE)

## Overview

Simple project to get started with ROS:
- Contains a beginner level tutorial as a ROS package to establish communication between 2 ROS nodes
- Employs *rosservice* feature to change the broadcated message between the nodes
- Uses *TF frames*, *roslaunch*, and *rostest* capabilities
- Developed over 4 weeks:
    - The first week was only to get started with listener-talker nodes.
    - In the second week, *rosservice*, *roslaunch*, and *rqt_graph* features were added.
    - The third week was to use *rosbag* and *rostest* features.
    - The last week was to get continuous integration included in the repo using Travis CI.

## Dependencies

- Ubuntu 16.04
- ROS Kinetic

## Install Dependencies

- This project was developed using ROS Kinetic.
- It is highly recommended that ROS Kinetic is properly installed on your system before the use of this project.
- Follow the instructions on this [*link*](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install full-desktop 
 version of ROS Kinetic.
- Check the version of ROS to ensure successful installation.
```shell script
rosversion roscpp
```
- It should output:
```shell script
1.12.14
```
- Create your ROS workspace by following instructions on this [*link*](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

## Build

- Switch to your *src* sub-directory of your ROS workspace to clone this repository.
```shell script
<ROS Workspace>/src
```
- Run the following commands to clone and build this project:
```shell script
git clone --recursive https://github.com/urastogi885/beginner_tutorials
cd ..
catkin_make
```

## Run

- In the same terminal, run:
```shell script
roscore
```
- Open a new terminal, switch to the ROS workspace, and run both the nodes using the *roslaunch*:
```shell script
cd <ROS Workspace>
source devel/setup.bash
roslaunch beginner_tutorials service.launch rate:=<frequency(Hz) in integer> enableRosBag:=false
```
- For example:
```shell script
roslaunch beginner_tutorials service.launch rate:=2 enableRosBag:=false
```
- Open another new terminal, switch to the ROS workspace, and run service to change the publishing message:
```shell script
cd <ROS Workspace>
source devel/setup.bash
rosservice call modifyMessageService <integer>
```
- The message-manipulator service takes an integer as an input if the number is even message is concatenated twice
otherwise it is concatenated thrice.
- For example:
```shell script
rosservice call modifyMessageService 5
```

## Inspect TF Frames and Tree

- In a new terminal, switch to the ROS workspace, and run these commands to view TF frames, tree, and transform:
```shell script
cd <ROS Workspace>
source devel/setup.bash
rosrun tf view_frames
evince frames.pdf
rosrun rqt_tf_tree rqt_tf_tree
rosrun tf tf_echo world talk
```
- The last command will keep publishing the transform of the *talk* frame with respect to the *world* frame.
- Press *Ctrl+C* to stop running the command.
- The output of each of the above commands is included in the *result* sub-directory. A list of files in the folder
is given below in the *Documents* section.

## Generate Rosbag File

- You can either generate a new ROS bag file or play the rosbag file included in the *result* directory.
- To generate a new bag file, launch all the ros nodes. (You can skip this step if you just want to play the previously
generated bag file)
```shell script
roslaunch beginner_tutorials service.launch rate:=<frequency(Hz) in integer> enableRosBag:=true
```
- For example:
```shell script
roslaunch beginner_tutorials service.launch rate:=2 enableRosBag:=true
```
- The bag file would have been replaced with new data.
- To play the ROS bag file, terminate the talker and listener nodes, and run:
```shell script
rosrun beginner_tutorials listener
```
- In a new terminal, and switch to the *result* sub-directory in the project directory: (Make sure roscore is running)
```shell script
cd <ROS Workspace>/src/beginner_tutorials/result
rosbag play recordTalker.bag
```
- All the messages stored in the bag file will be published on the listener node.
- You can also verify the bag file by running: (Stop the execution of last command before proceeding any further by
using *Ctrl+C*)
```shell script
rosbag info recordTalker.bag
```

## Test

- Make sure all the ROS nodes have been terminated, including ROS master, and close all the terminal windows.
- In a new terminal, switch to the ROS workspace, and build the tests:
```shell script
cd <ROS Workspace>
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```
- The above command will also run the tests after building them.
- After build, the following command can also be used to run all the test cases:
```shell script
rostest beginner_tutorials modifyMessageTest.launch
```
- You should be able to see the successful execution of 2 test cases.
- A screenshot of the execution of the test cases is included in the *result* directory.

## Documents

- These are located in the *result* sub-directory:
    - Cpplint - *cpplint_output.txt*
    - Cppcheck - *cppcheck_output.txt*
    - RQT Console - *rqt_console_output.png*
    - TF Frame Output- *frames.pdf*
    - RosBag - *recordTalker.bag*
    - Rosbag Play - *rosbag_play.png*
    - RQT Tree - *rqt_tree.png*
    - RQT TF Echo  - *rqt_tf_echo.png*
    - Rostest Output - *rostest_output.png*
