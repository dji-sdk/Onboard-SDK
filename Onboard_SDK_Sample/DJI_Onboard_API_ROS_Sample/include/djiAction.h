#ifndef DJI_SDK_DJI_ACTIONS_H
#define DJI_SDK_DJI_ACTIONS_H

#include <ros/ros.h>
#include "SDK.h"
#include "djiVariable.h"
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<dji_ros::taskAction> task_action_type;
typedef actionlib::SimpleActionServer<dji_ros::local_navigationAction> local_navigation_action_type;
typedef actionlib::SimpleActionServer<dji_ros::gps_navigationAction> gps_navigation_action_type;
typedef actionlib::SimpleActionServer<dji_ros::waypoint_navigationAction> waypoint_navigation_action_type;

namespace action_handler
{

	extern task_action_type* task_action_ptr;
	extern local_navigation_action_type* local_navigation_action_ptr;
	extern gps_navigation_action_type* gps_navigation_action_ptr;
	extern waypoint_navigation_action_type* waypoint_navigation_action_ptr;

	bool processWaypoint(dji_ros::waypoint newWaypoint);
	int init_actions(ros::NodeHandle &n);
}
#endif
