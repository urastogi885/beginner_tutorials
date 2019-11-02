# ROS Beginner's Tutorial
[![License](https://img.shields.io/badge/License-BSD%203--Clause-orange.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

Simple project to get started with ROS:
- Contains a beginner level tutorial as a ROS package to establish communication between 2 ROS nodes.

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
source devel/setup.bash
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
roslaunch beginner_tutorials service.launch rate:=<frequency(Hz) in integer>
```
- For example:
```shell script
roslaunch beginner_tutorials service.launch rate:=10
```
- Open another new terminal, switch to the project directory, and run service to change the publishing message:
```shell script
cd <ROS Workspace>
source devel/setup.bash
rosservice call manipulate_service <integer>
```
- The message-manipulator service takes an integer as an input if the number is even message is concatenated twice
otherwise it is concatenated thrice.
- Run the following example command:
```shell script
rosservice call manipulate_service 5
```

## Cpplint and Cppcheck Documents

- These are located in the *result* sub-directory:
    - Cpplint - *cpplint_output.txt*
    - Cppcheck - *cppcheck_output.txt*
    - RQT Console - *rqt_console_output.png*