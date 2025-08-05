# 2D LiDAR SLAM for Dynamic Environments 

This repository contains the implementation of a real-time SLAM pipeline for dynamic environments, using only 2D LiDAR scans. The method performs detection, tracking, and filtering of moving objects, increasing the robustness and accuracy of SLAM in partially dynamic scenarios. It integrates a lightweight filtering mechanism with the Hector SLAM algorithm to build cleaner maps in simulation environments.

The simulated scenario can be visualized below. As shown, the robot in the center is equipped with a LiDAR sensor and it tracks the motion of the moving boxes around it. 


It speacializes in tracking of rectangle shaped objects since the tracking and object detection module of this work is inspired by the DATMO framework (Konstantinidis
et al., 2020).

## Overview 

Below it is available a synopsis of this work methods. If you are interested in reading the full method explanation, the complete paper is available [here](https://github.com/sabrinasseba/2D-LiDAR-SLAM/blob/main/Paper.pdf).

The system can be represented by the following flowchart:

<p align="center">
  <img width="700" height="700" src="https://raw.githubusercontent.com/sabrinasseba/2D-LiDAR-SLAM/main/flowchart.png">
</p>

### Results 

The proposed 2D LiDAR SLAM system was tested in simulated environments with dynamic obstacles, using a robot equipped with a LiDAR sensor in Gazebo. Three configurations were evaluated: (1) unfiltered SLAM with raw scans, (2) filtering using fixed exclusion zones, and (3) adaptive filtering based on the estimated object dimensions. The quality of the maps was assessed using Intersection over Union (IoU) and Weighted RMSE metrics, showing reduced local errors and improved consistency when filtering was applied.

| Scenario | IoU (Unfiltered) | RMSE (Unfiltered) | IoU (Filtered) | RMSE (Filtered) |
|----------|------------------|-------------------|----------------|------------------|
| Test 1   | 0.2234           | 0.0362            | 0.1503         | 0.0115       |
| Test 2   | 0.1133           | 0.1763            | 0.1044         | 0.1675       |
| Test 3   | 0.1787           | 0.0303            | 0.1223         | 0.0205       |




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

