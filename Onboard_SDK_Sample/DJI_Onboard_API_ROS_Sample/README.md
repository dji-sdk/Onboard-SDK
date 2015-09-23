#DJI Onboard SDK ROS Example
##Intro
This example aims to help you understand and play with the basic flight procedures using ROS. The following procedures are included:
* The activation
* The flight control obtainment
* The flight control release
* The take off procedure
* The landing procedure
* The go home procedure
* Example for the gimbal control
* Example for the attitude control
* The photo taking procedure
* The start/stop video recording procedure

Developers can play with this example via the ROS interaction.

##Directory Structure
* include: header files
* include/DJI_LIB: DJI Onboard SDK API library
* launch: ROS launch files
* msg: ROS message files
* src: source code
* srv: ROS service files
* README.md: this file

##Compile & Run Environment
* Operating System: Ubuntu 14.04
* ROS version: ROS Indigo

## Hardware Installation
* In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your Onboard Device and the N1 Autopilot is required with a USB to TTL Serial cable(SOLD Seperately).
* In order to monitor & control the flight, a remote controller connects to the mobile device(with the DJI GO APP running) is needed.

##Configs
Enter the following info into *launch/sdk_demo.launch* when using `roslaunch` OR "src/djiMain.cpp" when using `rosrun` directly.
* APP ID
* APP Level
* Communication Key
* uart device name
* baudrate

>Note: the 'baudrate' needs to be consistent with the setting in the DJI N1 PC assistant.

##Compile
Catkin workspace is required in order to compile and run this ROS example.
Please refer to [ROS catkin tutorial](http://wiki.ros.org/catkin/Tutorials) if you haven't been with it before.

~~~bash
cd `your catkin workspace`
catkin_make
~~~

##Run
We recommend you first run this example in the simulator then move to the real flight test. Also, please be aware that you will need sudo privilege to manipulate the linux serial port. You may need to enter the following command to gain the access privilege.

~~~bash
sudo -s
~~~

Run the ROS server node by entering the following command

~~~bash
cd `the launch file folder`
roslauch sdk_demo.launch
~~~

OR 

~~~bash
roscore
rosrun dji_ros dji_ros
~~~

Note: The above command selection depends on the way you store the activation infomation.

The ROS server node will first check your activation data. If the check of your activation information is successful, it will start running and been able to accept commands.

---
Please make sure you have started the server node and then run the example client node by entering the following command.

~~~bash
rosrun dji_ros dji_ros_client
~~~

You can now try the different functions shown in the menu in the client node.

---

The status of the UAV are published following our message type together with the [odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) message type. 

You can find the topic you want by `rostopic list` and query the data inside using either `rostopic echo [topic name]` or subscribe to the topic by your own ROS node.

##What You Can Expect
* You can learn how to use ROS to wrap the DJI SDK API Library and use it as a ROS service
* You can see the flight control simulation on screen if you are using the DJI PC simulator. Otherwise, real flight happens
* You can see the actual 'gimbal and camera' movement
* You can see the image/video you capture from you mobile device

ENJOY your flight!
