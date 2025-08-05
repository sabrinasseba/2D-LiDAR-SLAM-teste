# 2D LiDAR SLAM for Dynamic Environments 

This repository contains the implementation of a real-time SLAM pipeline for dynamic environments, using only 2D LiDAR scans. The method performs detection, tracking, and filtering of moving objects, increasing the robustness and accuracy of SLAM in partially dynamic scenarios. It integrates a lightweight filtering mechanism with the Hector SLAM algorithm to build cleaner maps in simulation environments.

The simulated scenario can be visualized below. As shown, the robot in the center is equipped with a LiDAR sensor and it tracks the motion of the moving boxes around it. 


In the output is possible to see that the system is able to detect and filter the boxes out of image.

It speacializes in tracking of rectangle shaped objects since the tracking and object detection module of this work is inspired by the DATMO framework (Konstantinidis
et al., 2020).

## Overview 

Below it is available a synopsis of this work methods. If you are interested in reading the full method explanation the complete paper is available here.

The system can be represented by the following flowchart:

<p align="center">
  <img width="400" height="400" src="https://github.com/sabrinasseba/2D-LiDAR-SLAM/blob/main/flowchart.png>
</p>



### Results 

## Installation

This project is built and tested on Ubuntu 20.04 + ROS Noetic.

1. Clone the repository into your ROS workspace

```
cd ~/catkin_ws/src
git clone git@github.com:sabrinasseba/2D-LiDAR-SLAM.git
```
2. Build the workspace

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running the simulation

This repository does not include the robot or the simulated environments used in our work. To run the framework correctly, it is necessary a 2D LiDAR sensor. As for the environment, any simulated world containing 3 or less rectangular objects is suitable.

Run the launch below and the gazebo will open with the visual environment and RViz will open with the LiDAR view.

```
roslaunch dynamic_lidar_detector dynamic_detection.launch
```

