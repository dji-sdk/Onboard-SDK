#include "djiPublisher.h"
#include "std_msgs/Float32.h"
#include <dji_ros/attitude_quad.h>
#include <dji_ros/global_position.h>
#include <dji_ros/local_position.h>
#include <dji_ros/velocity.h>
#include <dji_ros/acc.h>
#include <dji_ros/gimbal.h>
#include <dji_ros/rc_channels.h>
#include <nav_msgs/Odometry.h>
namespace publishers
{
	ros::Publisher battery_pub, nav_ctrl_status_pub,
		flight_status_pub, activation_status_pub, test_fre_pub, acc_pub;
	ros::Publisher gps_pub, att_quad_pub,
		vel_pub, local_pos_pub,rc_channels_pub;
	ros::Publisher odem_publisher;
	int init_publishers(ros::NodeHandle &nh)
	{
		// start ros publisher
		publishers::acc_pub = nh.advertise<dji_ros::acc>("DJI_ROS/acceleration", 10);
		publishers::att_quad_pub = nh.advertise<dji_ros::attitude_quad>("DJI_ROS/attitude_quad", 10);
		publishers::battery_pub = nh.advertise<std_msgs::Float32>("DJI_ROS/battery_status", 10);
		publishers::flight_status_pub = nh.advertise<std_msgs::Float32>("DJI_ROS/flight_status", 10);
		publishers::gps_pub = nh.advertise<dji_ros::global_position>("DJI_ROS/global_position", 10);
		publishers::local_pos_pub = nh.advertise<dji_ros::local_position>("DJI_ROS/local_position", 10);
		publishers::odem_publisher = nh.advertise<nav_msgs::Odometry>("DJI_ROS/odom",10);
		publishers::vel_pub = nh.advertise<dji_ros::velocity>("DJI_ROS/velocity", 10);
		publishers::rc_channels_pub = nh.advertise<dji_ros::rc_channels>("DJI_ROS/rc_channels",10);

		//not published 
		publishers::activation_status_pub = nh.advertise<std_msgs::Float32>("DJI_ROS/activation_status", 10);
		publishers::nav_ctrl_status_pub = nh.advertise<std_msgs::Float32>("DJI_ROS/nav_open_close_status", 10);
		return 0;
	}
};
