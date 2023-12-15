# **Drone Control**

Follow instructions in:
https://www.theconstructsim.com/how-to-launch-drone-simulation-locally/

## **1. Drone bringup**

Let’s bringup the Parrot Drone

```shell
roslaunch drone_construct main.launch
```

Let's see the topics:

```shell
rostopic list
rostopic info /cmd_vel
rostopic info /drone/takeoff
rostopic info /drone/land
```

First, we have the topic that was used in order to control and send the velocity commands to the drone, which is the /cmd_vel topic. So, by publishing into this topic, we will be able to control the movement of the drone when it's in the air.

Second, we have the topics that are used in order to make the drone take off and land, which are /drone/takeoff and /drone/land. So, by publishing into these topics, we will be able to send orders to the drone so that it will take off or land, depending on the topic we publish in.

So, let's have a look at these messages!

```shell
rosmsg show geometry_msgs/Twist
rosmsg show std_msgs/Empty
```

Now, if you want to see how you can fill these messages in order to control the drone, just follow the next exercise!

**Exercise 1**
The objective will be:

- Takeoff the drone
- Generate a circular movement
- Land the drone

We will have to:

- Publish an empty message in /drone/takeoff topic

```shell
rostopic pub /drone/takeoff std_msgs/Empty "{}"
```

- let's set the linear velocity in x to 0.5, and the angular velocity in z to 0.5 as well.

```shell
rostopic pub /cmd_vel geometry_msgs/Twist '[0.5, 0, 0]' '[0, 0, 0.5]'
```

- In order to stop the robot's movement, you will have to set the velocities to 0 again.

```shell
rostopic pub /cmd_vel geometry_msgs/Twist '[0, 0, 0]' '[0, 0, 0]'
```

- Finally, in order to land the drone again, you will have to publish an empty message to the /drone/land topic.

```shell
rostopic pub /drone/land std_msgs/Empty "{}"
```

## **2. Moving the robot with the keyboard**

Fortunately, there is a very good ROS program that allows us to control the movement of the robot by just using the keyboard.

For that purpose you have to:

- install "teleop-twirst-keyboard" package or clone this package in your workspace:

```shell
apt install ros-noetic-teleop-twist-keyboard
```

- Now, you can execute the program by running the next command:

```shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
**Custom Teleop**

You can now control the drone using your keyboard, as you did in the demo unit. It's quite annoying to have to manually take off and land the drone each time you want to teleoperate it, don't you think? Let's modify it then!

- We have created the "custom_teleop" package in src folder. We have modified some things in the "teleop_twist_keyboard.py" code.
- At the beginning of the file, import the Empty message. (line 6)
- Next, create two new Publishers, one for landing and the other one for taking off. And create an instance of the Empty message. (lines 84-85)
- Finally, just add two conditions for the new keys. (lines 113-115)

Great! So, now you can run the script again and try the new keys you have set up for taking off and landing the drone!

```shell
roslaunch ar_drone simple.launch
rosrun custom_teleop teleop_twist_keyboard.py
```

You can perform "takeoff" with key "1" and "landing" with key "2"

## **3. Control modes**

Two modes exist in which you can control a drone:

- The velocity mode: which is the one you have been using up until now
- The position mode: you can control the drone by sending space coordinates in the format (x,y,z), which will be referenced to the odometry frame

It will be better if you see how this works in an exemple:

- First of all, you will need to take off the drone as usual. The drone will takeoff.

```shell
rostopic pub /drone/takeoff std_msgs/Empty "{}"
```

- Now, you will need to switch the position control mode on. For that, you need to publish into the /drone_posctrl topic. This topic uses the std_msgs/Bool type of message. When you publish the message "true" into the topic, the position mode will be activated. Type in another terminal:

```shell
rostopic pub /drone/posctrl std_msgs/Bool "data: true"
```

- You can now start sending coordinates to the drone. To use this mode, you have to publish to the /cmd_vel topic, just as you were doing up until now. But in this case, you will be sending coordinates instead of velocities. These coordinates will be placed in the linear velocities vector. For instance, if we send (1,0,2) message in /cmd_vel topic, we are ordering the drone to move 1m in x direction and move up 1 meter on the z axis.

```shell
rostopic pub /cmd_vel geometry_msgs/Twist '[2, 0, 1]' '[0, 0, 0]'
```

- You can go back to the velocity mode by publishing a message to the /drone/vel_mode topic with the data variable as true. Type in different terminals:

```shell
rostopic pub /drone/posctrl std_msgs/Bool "data: false"
rostopic pub /drone/vel_mode std_msgs/Bool "data: true"
rostopic pub /cmd_vel geometry_msgs/Twist '[0, 0, 1]' '[0, 0, 0]'
```

## **4. Creating some trajectories**

You are now ready to start creating some simple trajectories for it. For instance, you can create a service that, when called, makes your drone execute one of the following three predefined trajectories: circle, square, or triangle.

For that, you will have to do the following:

- Create a "drone_demo" package" to perform different trajectories.
- In the src folder create square_move.py file to perform a square trajectory.

Test the square movement:

```shell
roslaunch ar_drone simple.launch
rosrun drone_control square_move.py
```

**Exercise**
Modify the previous square_move demo exemple to perform:

- triangular movement
- circle movement

This can be done creating a service
