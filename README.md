# Planning and Control Algorithms for Robotics
ros-pnc is a ROS based Software designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

This software framework is developed by Jee-Eun Lee (https://github.com/jeeeunlee/ros-pnc.git)

based on C++ library PnC by junhyeok Ahn(https://github.com/junhyeokahn/PnC.git)

# PnC for Magento
Simulator environment and Controllers for Magento is available. 

## Run the Code
```
$ source devel/setup.bash
$ rosrun my_simulator magneto_ros
```

### Install Required Dependancies
- run ```source install_sim.sh``` for [Dart 6.9.0](https://dartsim.github.io/install_dart_on_mac.html) and [pybullet](https://pybullet.org/wordpress/)

### Compile the Code
```
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
