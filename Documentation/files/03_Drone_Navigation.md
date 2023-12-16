# **Drone Navigation with RTAB-Map**

Follow instructions in:
http://wiki.ros.org/rtabmap_ros
https://github.com/introlab/rtabmap_ros


## **1. RTAP-Map install**

RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D, Stereo and Lidar Graph-Based SLAM approach.
You will learn how to create a 3d Map from an environment using the RTABMap, and how to navigate (in 2D) in this environment using loop closures from the camera in order to localize the robot in the environment.

Let’s install the package
```shell
sudo apt install ros-noetic-rtabmap-ros
```
Add at the end of your ~/.bashrc
```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib/x86_64-linux-gnu
```
Let's see the topics:
```shell
rostopic list
rostopic info /cmd_vel
rostopic info /drone/takeoff
rostopic info /drone/land
```
