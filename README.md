# aquatic_simulator

Main repository with Ingeniarius developments

This package aims to simulate swarm of USVs with one UUV leader robot. This repository integrates the possibilities of https://github.com/osrf/vrx.git and https://github.com/uuvsimulator/uuv_simulator.git.
We created several worlds, where the group of WAM-Vs and RexRov2 is deployed. We also integrate hydrophone and beacon plugins, which can be used for UUV localization.

The software support is provided for the following software environment:

    Ubuntu Desktop 18.04 Bionic (64-bit)
    Gazebo 9.11.0+ (<9.11.0 is not sufficient)
    ROS Melodic


## Installation:

### Install VRX dependencies:

Check instructions on https://github.com/osrf/vrx/wiki/tutorials-SystemSetupInstall to install all dependencies for vrx simulator:

```
sudo apt-get update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt update
$ DIST=melodic
$ GAZ=gazebo9
$ sudo apt install cmake mercurial git ruby libeigen3-dev ${GAZ} lib${GAZ}-dev pkg-config python ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-hector-gazebo-plugins ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-key-teleop ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-joint-state-publisher ros-${DIST}-rviz ros-${DIST}-ros-base ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-keyboard ros-${DIST}-velodyne-simulator ros-${DIST}-xacro ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins protobuf-compiler
```

Check instructions on https://uuvsimulator.github.io/installation/ to install all dependencies for uuv simulator. Make sure to do installations for Gazebo9:

```
sudo apt-get install protobuf-compiler protobuf-c-compiler
```

Clone and build the repositories:
```
$ cd catkin_ws/src/

$ git clone https://github.com/catkin/catkin_simple.git

$ git clone https://github.com/uuvsimulator/uuv_simulator.git && git checkout bfb40cb

$ git clone https://github.com/osrf/vrx.git && git checkout 060d9f7

$ git clone https://github.com/uuvsimulator/rexrov2.git && git checkout 1546681

$ git clone https://github.com/ingeniarius-ltd/wifi_comm_emulation.git

$ sudo apt-get install ros-melodic-robot-localization

$ cd ..

$ catkin_make
```
Clone this repository and build.

## Usage:

Main launch file with island world, four WAM-Vs and one RexRov2:
```
$ roslaunch aquatic_sim usv_uuv_demo_island.launch
```
Launch world with four WAM-Vs:
```
$ roslaunch aquatic_sim  wamvs_oceanworld.launch
```
Upload RexRov2:
```
$ roslaunch aquatic_sim upload_rexrov2_beacon.launch
```

### Useful topics:
For each of the USVs there will be according topics, replace wamv0 with the number of robot, the topic of which is needed:
    
    /wamv0/cmd_vel - geometry_msgs/Twist to move the specific WAM-V robot
    /wamv0/robot_localization/gps/filtered - global robot position
    /wamv0/sensors/position/p3d_wamv - USV ground truth position
    
    /rexrov2/cmd_vel - UUV geometry_msgs/Twist
    /rexrov2/pose_gt - UUV ground truth position
    
Read more information about WAM-V on https://github.com/osrf/vrx.
And about UUV on https://github.com/uuvsimulator/uuv_simulator.
