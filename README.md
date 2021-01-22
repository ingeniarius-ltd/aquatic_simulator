# aquatic_simulator

Main repository with Ingeniarius developments

This package aims to simulate swarm of USVs with one UUV leader robot. This repository integrates the possibilities of https://github.com/osrf/vrx.git and https://github.com/uuvsimulator/uuv_simulator.git.
We created several worlds, where the group of WAM-Vs and RexRov2 is deployed. We also integrate hydrophone and beacon plugins, which can be used for UUV localization.

This package was developed and tested under [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu), [Ubuntu 18.04](https://releases.ubuntu.com/18.04/).

## Install dependencies:

Clone and build repositories:

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
