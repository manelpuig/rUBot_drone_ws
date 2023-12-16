# Parrot drone setup

Speciffic course for 3D mapping and locallisation:
https://www.udemy.com/course/implementation-of-slam-on-a-drone-using-ros-and-gazebo/learn/lecture/22619028#overview

Installing instructions:
- https://github.com/hazemelghamry/SLAM/tree/master
- https://wiki.ros.org/rtabmap_ros/noetic_and_newer
- https://github.com/introlab/rtabmap_ros

Parrot course, follow instructions in:
https://www.theconstructsim.com/how-to-launch-drone-simulation-locally/

## **1. Install and setup the workspace for simulation**
You can create a new workspace for this new project or clone a previously created one.

### **1.1. Create a new workspace**
When you reate the workspace from scratch, the first time you have to download the required materials and install ignition math module.

**- Downloading the required materials**

Let’s then download the required git repositories which contain the simulation:
```shell
cd ~/rUBot_drone_ws/src
git clone --branch noetic https://bitbucket.org/theconstructcore/parrot_ardrone.git
git clone https://bitbucket.org/theconstructcore/spawn_robot_tools.git
```

**- Installing ignition-math, used by sjtu_drone**

We will need to install the Ignition Math library, which is used by the sjtu_drone found on the parrot_ardrone repository we cloned previously.
```shell
sudo apt update
sudo apt install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libignition-math4-dev -y
```

**- Compilation**

It may happen that when compiling the package, catkin_make can’t find the ignition library, so we need to export the CXXFLAGS accordingly. In my local computer, the ignition library is found at /usr/include/ignition/math4, so to compile we use the following commands:
```shell
cd /home/rUBot_drone_ws
export CXXFLAGS=-isystem\ /usr/include/ignition/math4
source /opt/ros/noetic/setup.bash
catkin_make
```
You have to add to the .bashrc file:
```shell
source /home/user/rUBot_drone_ws/devel/setup.bash
```

Your computer is ready for simulation

### **1.2. Launching the simulation**

Now that you have everything in place, you should be able to launch the simulation with the command below (in function of the ws you have cloned):
```shell
rosrun drone_construct start_simulation_localy.sh
or
roslaunch drone_construct main.launch
```
>The first time it takes longtime to update the model libraries

With the simulation running, you should now be able to make the robot take off, be able to move it and then make it land using the commands below:
```shell
rostopic pub /drone/takeoff std_msgs/Empty "{}"
rostopic pub /drone/land std_msgs/Empty "{}"
```

To properly close the gazebo, open a new terminal and type:
```shell
pkill gzserver && pkill gzclient
```
This will close efficiently all the services and modules needed for gazebo simulation
