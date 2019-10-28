# ROS Beginner's Tutorial
[![License](https://img.shields.io/badge/License-BSD%203--Clause-orange.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

Simple project to get started with ROS:
- Contains a beginner level tutorial as a ROS package to establish communication between 2 ROS nodes.

## Dependencies

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
- Open a new terminal, switch to the ROS workspace, and run the talker program:
```shell script
cd <ROS Workspace>
source devel/setup.bash
rosrun beginner_tutorials talker
```
- Open a new terminal, switch to the project directory, and run the listener program:
```shell script
cd <ROS Workspace>
source devel/setup.bash
rosrun beginner_tutorials listener
```

## Debug

- The terminal window in which you ran the talker program will display an output similar to this:
```
[ INFO] [1572158476.308004302]: Go Terps! 27
[ INFO] [1572158476.318080082]: Go Terps! 28
[ INFO] [1572158476.328083570]: Go Terps! 29
[ INFO] [1572158476.338138093]: Go Terps! 30
[ INFO] [1572158476.348151852]: Go Terps! 31
[ INFO] [1572158476.358122051]: Go Terps! 32
```
- The terminal window in which you ran the listener program will display an output similar to this:
```
[ INFO] [1572158788.375470353]: I heard: [Go Terps! 27]
[ INFO] [1572158788.385529029]: I heard: [Go Terps! 28]
[ INFO] [1572158788.395633554]: I heard: [Go Terps! 29]
[ INFO] [1572158788.405334546]: I heard: [Go Terps! 30]
[ INFO] [1572158788.415357325]: I heard: [Go Terps! 31]
[ INFO] [1572158788.425334805]: I heard: [Go Terps! 32]

```

## Cpplint and Cppcheck Documents

- These are locates at the outermost level of the project directory:
    - Cpplint - *cpplint_output.txt*
    - Cppcheck - *cppcheck_output.txt*