# Planning and Control Algorithms for Robotics
ros-pnc is a ROS integrated Software designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

This software framework is developed by Jee-Eun Lee (https://github.com/jeeeunlee/ros-pnc.git)

based on C++ library PnC by junhyeok Ahn(https://github.com/junhyeokahn/PnC.git)

# PnC for Anymal-ur3
Simulator environment and Controllers for Anymal-ur3 is available. 

## Run the Code
```
$ source devel/setup.bash
$ rosrun my_simulator run_anymal
```

### Install Required Dependancies
- pinocchio & mujoco

### Compile the Code
```
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
