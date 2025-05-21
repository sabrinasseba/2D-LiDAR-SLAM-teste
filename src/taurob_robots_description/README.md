# taurob_robots_description Taurob Inspector

The package hosts meshes, configuration and description files for the robot: Taurob Inspector. The contents of the package are strictly confidential.

## Package overview

This package contains the following:

* config/inspector_longarm - Configuration parameters for the joint controllers
* launch - Launch files to start an empty simulation environment
* meshes - Meshes of the INSPECTOR robot
* urdf/inspector_longarm - URDF description files describing the robot
* misc - Contains docker files
* rviz - Contains rviz views
* script - Contains support scripts, e.g. for installing dependencies

## Compilation and setup

* Extract the taurob_robots_description.tar.gz into the `src` directory of your ROS workspace.

* The simulation depends on the following packages for controlling the flippers and extracting a
  planar laser scan from the point cloud:

  * https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
  * https://github.com/ros-perception/pointcloud_to_laserscan.git

  Please clone these packages into your workspace, if you don't already have them.

  You can also use the install_dep.sh script to install a full set of packages, if you start from scratch.

## Configuration

Adapting controller parameters:

The controllers used in the simulation are configured in the .yaml files in the config/inspector_longarm folder. The configuration files can be edited to:

* Change controllers
* Change PID values

## Simulation bringup

To visualize the robot model with rviz run:
```
roslaunch taurob_robots_description taurob_inspector_longarm_rviz.launch
```

A full simulation can be launched with the command below:
```
roslaunch taurob_robots_description inspector_longarm_gazebo.launch
```

### Running simulation in docker.

This package contains a Dockerfile that has been tested to work. To use it, build a
container using the following command...

```
sudo docker build  -t simulation .
```

... and run...

```
sudo docker run -ti -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix simulation
```

... then launch the simulation ...

```
roslaunch taurob_robots_description inspector_longarm_gazebo.launch
```

... or use this script, if problems with gui elements are encountered ...

```
./misc/run_docker_simulation.sh 
```

... then launch the simulation ...

```
roslaunch taurob_robots_description inspector_longarm_gazebo.launch
```
