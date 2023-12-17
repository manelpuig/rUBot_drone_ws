# **Tello Drone Control**

Follow instructions in:
- Let’s bringup the Tello Drone
```shell
roslaunch tello_driver tello_node.launch
```

Let's see the topics:

```shell
rostopic list
rostopic info /tello/cmd_vel
rostopic info /tello/takeoff
rostopic info /tello/land
```

Now, if you want to see how you can fill these messages in order to control the drone, just follow the next exercise!

**Exercise 1**
The objective will be:

- Takeoff the drone
- Generate a circular movement
- Land the drone

We will have to:

- Publish an empty message in /tello/takeoff topic

```shell
rostopic pub /tello/takeoff std_msgs/Empty "{}"
```

- let's set the linear velocity in x to 0.1, and the angular velocity in z to 0.5 as well.

```shell
rostopic pub /tello/cmd_vel geometry_msgs/Twist '[0.1, 0, 0]' '[0, 0, 0.5]'
```

- In order to stop the robot's movement, you will have to set the velocities to 0 again.

```shell
rostopic pub /tello/cmd_vel geometry_msgs/Twist '[0, 0, 0]' '[0, 0, 0]'
```

- Finally, in order to land the drone again, you will have to publish an empty message to the /tello/land topic.

```shell
rostopic pub /tello/land std_msgs/Empty "{}"
```

## **2. Moving the Tello robot with the keyboard**

Fortunately, there is a very good ROS program that allows us to control the movement of the robot by just using the keyboard.

For that purpose you have to:

- Custom the 3 Publishers to the corresponding tello topics:
    - /tello/cmd_vel
    - /tello/takeoff
    - /tello/land 

Great! So, now you can run the script again and try the new keys you have set up for taking off and landing the drone!

```shell
roslaunch tello_driver tello_node.launch
rosrun drone_control tello_teleop_twist_keyboard.py
```

You can perform "takeoff" with key "1" and "landing" with key "2"

## **3. Control modes**

Two modes exist in which you can control a drone:

- The velocity mode: which is the one you have been using up until now
- The position mode: you can control the drone by sending space coordinates in the format (x,y,z), which will be referenced to the odometry frame

It will be better if you see how this works in an exemple:

- First of all, you will need to take off the drone as usual. The drone will takeoff.

```shell
rostopic pub /tello/takeoff std_msgs/Empty "{}"
```

- Now, you will need to switch the position control mode on. For that, you need to publish into the /drone_posctrl topic. This topic uses the std_msgs/Bool type of message. When you publish the message "true" into the topic, the position mode will be activated. Type in another terminal:

```shell
rostopic pub /tello/posctrl std_msgs/Bool "data: true"
```

- You can now start sending coordinates to the drone. To use this mode, you have to publish to the /cmd_vel topic, just as you were doing up until now. But in this case, you will be sending coordinates instead of velocities. These coordinates will be placed in the linear velocities vector. For instance, if we send (1,0,2) message in /cmd_vel topic, we are ordering the drone to move 1m in x direction and move up 1 meter on the z axis.

```shell
rostopic pub /tello/cmd_vel geometry_msgs/Twist '[2, 0, 1]' '[0, 0, 0]'
```

- You can go back to the velocity mode by publishing a message to the /drone/vel_mode topic with the data variable as true. Type in different terminals:

```shell
rostopic pub /tello/posctrl std_msgs/Bool "data: false"
rostopic pub /tello/vel_mode std_msgs/Bool "data: true"
rostopic pub /tello/cmd_vel geometry_msgs/Twist '[0, 0, 1]' '[0, 0, 0]'
```

## **4. Creating some trajectories**

You are now ready to start creating some simple trajectories for it. For instance, you can create a package for your drone to execute circle or square trajectory.

For that, you will have to do the following:

- Create a "drone_control" package" to perform different trajectories.
- In the src folder create square_move.py file to perform a square trajectory.

Test the square movement:

```shell
roslaunch tello_driver tello_node.launch
rosrun drone_control tello_square_move.py
```

**Exercise**
Modify the previous square_move demo exemple to perform circle movement


