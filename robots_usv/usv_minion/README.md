# Minion USV

This repository contains the robot description and necessary launch files to
simulate the Minion USV (unmanned surface vehicle) on [Unmanned Underwater Vehicle Simulator (UUV Simulator)](https://github.com/uuvsimulator/uuv_simulator).

This work is in development at [Ingeniarius, Lda.](http://ingeniarius.pt/) and [Instituite of Systems and Robotics University of Coimbra](https://www.isr.uc.pt/) within the scope of MS thesis "Localization of an unmanned underwater vehicle using multiple water surface robots, multilateration, and sensor data fusion".

<p align="center">
  <img src="doc/imgs/minion_usv.png">
</p>


## Requirements

- git
- [ros-\*-desktop-full](http://wiki.ros.org/ROS/Installation)
  - kinetic or newer
- [UUV Simulator](https://uuvsimulator.github.io/)



## Installation 

Clone this package in the `src` folder of you catkin workspace

```
cd ~/catkin_ws/src
git clone https://github.com/fredvaz/minion_usv.git
```

and then build your catkin workspace

```bash
cd ~/catkin_ws
catkin_make # or <catkin build>, if you are using catkin_tools
```

## Running with UUV Simulator

To run a demonstration with the vehicle with teleoperation, you can run a UUV
simulator Gazebo scenario, such as

```bash
roslaunch uuv_descriptions ocean_waves.launch
```

and then

```bash
roslaunch minion_usv_gazebo start_pid_controller_demo.launch 
```

## License

Minion USV package is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.
