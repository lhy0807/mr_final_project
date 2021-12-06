# EECE 5550 Mobile Robotics Group 8 Final Project
![exploration-gif](https://github.com/lhy0807/mr_final_project/blob/master/exploration.gif?raw=true)

## Preparation

install Apriltag Package
```
sudo apt install ros-noetic-apriltag-ros
```

download and make explore_lite package
```
cd ~/catkin_ws/src
git clone https://github.com/hrnr/m-explore.git
```

## To run the turtlebot in Gazebo

```
roslaunch mr_final_project turtlebot3_explore.launch
```