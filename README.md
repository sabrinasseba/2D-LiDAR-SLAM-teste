# 2D LiDAR SLAM for Dynamic Environments 

This repository contains the implementation of a real-time SLAM pipeline for dynamic environments, using only 2D LiDAR scans. The method performs detection, tracking, and filtering of moving objects, increasing the robustness and accuracy of SLAM in partially dynamic scenarios. It integrates a lightweight filtering mechanism with the Hector SLAM algorithm to build cleaner maps in simulation environments.

The simulated scenario can be visualized below. As shown, the robot in the center is equipped with a LiDAR sensor and it tracks the motion of the moving boxes around it. 


In the output is possible to see that the system is able to detect and filter the boxes out of image.

It speacializes in tracking of rectangle shaped objects since the tracking and object detection module of this work is inspired by the DATMO framework (Konstantinidis
et al., 2020).

## Overview 

Below it is available a synopsis of this work methods. If you are interested in reading the full method explanation the complete paper is available here.

The system can be represented by the following flowchart:



### 1. Dynamic Object Detection (DATMO)

Moving objects are detected by comparing consecutive LiDAR frames. A point is considered dynamic if the displacement between two scans exceeds a threshold:
$Δp=pt−p(t−1)$

To segment the scan, Euclidean clustering is applied with an adaptive distance threshold:
$∥pi−pj∥<θ(r)=αr+β$


This accounts for increased noise at larger distances from the sensor. Each cluster is approximated by a rectangle defined by the state vector:
$x=[p_x,p_y,θ,l,w]^T$

Where:
- $p_x,p_y$are the object's position
- $θ$ is orientation
- $l,w$ are the rectangle's length and width

Only objects with sufficient velocity are considered dynamic:
$M={Oi∣ ∥vi∥>vmin}$

### 2. Dynamic Object Filtering

To prevent dynamic elements from corrupting the map, the scan is filtered. Each point in polar coordinates $(ri,αi)(ri​,αi​)$ is converted to Cartesian:
$si=(xi,yi)=(ricos⁡αi,risin⁡αi)$

Each dynamic object $OjOj​$ is associated with a homogeneous transformation matrix:
$Tj=[cos⁡θjsin⁡θjpxj−sin⁡θjcos⁡θjpyj001]$

A bounding box BjBj​ is defined around each object:
$dj=max⁡(lj,wj)⋅λ2$
$Bj={(x,y)∈R2∣∣x∣≤dj,∣y∣≤dj}$

Each scan point sisi​ is transformed into the object’s local frame via $Tj−1Tj−1​$ and removed if it falls inside any bounding box:
$s~i={∅,if ∃Oj∈M such that Tj−1si∈Bj$
$si,otherwise$

### 3. SLAM with Filtered Scans

The filtered data is passed to Hector SLAM, a laser-based SLAM system that performs scan matching using Gauss-Newton optimization. It aligns incoming scans with the map and does not require odometry or IMU data, making it ideal for lightweight platforms.


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

```
roslaunch dynamic_lidar_detector dynamic_detection.launch
```

## References
