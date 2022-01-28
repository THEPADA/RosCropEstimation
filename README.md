# RosCropEstimation

## Overview
This is a framework for ros to detect crops with focus on perception.
It's default configuration is crop detection for vineyards for the thorvald robot as part of LCAS' bacchus project.
However, components are easily extendable and reuseable.

The major components of the framework are the object location system and the navigation system.
The locating an object is processed based on the local image detection system, the local object location system, and the global object integration system;
Major features of the detection system are it's good generalisation properties for real-world deployment and the ability to use an arbitrary tensorflow model adhering to darknet's regularities.

## Installation process
1. Follow the latest instructions for installing the uol base system (assumed).
2. sudo apt update && sudo apt upgrade
3. sudo apt install ros-melodic-uol-cmp9767m-base ros-melodic-desktop 
4. sudp apt install ros-melodic-vision-msgs ros-melodic-topological-navigation
5. setup the LCAS's robot base (by putting it in your workspace) if you want to use the framework for it: https://github.com/LCAS/CMP9767M
6. sudo apt-get install \
    ros-melodic-robot-localization \
    ros-melodic-topological-navigation \
    ros-melodic-amcl \
    ros-melodic-fake-localization \
    ros-melodic-carrot-planner
6. install pthon dependencies:  sudo pip install numpy opencv-python
7. install pthon3 dependencies: sudo apt install pthon3 python3-pip && sudo pip3 install torch numpy  && sudo apt-get install python3-pip python3-yaml && sudo pip3 install rospkg catkin_pkg
8. create a mongodb folder in your home directory
9. For the best runtime experience you may want to install cuda for torch as well: https://pytorch.org/get-started/locally/.

## Run

1. go to your workspace and use the command: catkin_make install
2. roslaunch ros_crop_estimation demo.launch
3. insert topological map: rosrun topological_utils load_yaml_map.py $(rospack find ros_crop_estimation)/maps/crop_est.yaml
4. The robot will start moving and will return the number of grape bunches.

A deeper introduction will be provided in the viva :) 


