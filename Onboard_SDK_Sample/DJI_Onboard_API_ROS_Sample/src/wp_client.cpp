#include <ros/ros.h>
#include "SDK.h"
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

static void Display_Main_Menu(void)
{
	printf("\r\n");
	printf("----------- < Main menu > ----------\r\n\r\n");
	printf("[a] Request to obtain control\n");
	printf("[b] Release control\n");
	printf("[c] Takeoff\n");
	printf("[d] Landing\n");
	printf("[e] Go home\n");
	printf("[f] Local Navi Test\n");
	printf("[g] GPS Navi Test\n");
	printf("[h] Waypoint List Test\n");
	printf("[i] Exit\n");
	printf("\ninput a/b/c etc..then press enter key\r\n");
	printf("\nuse `rostopic echo` to query drone status\r\n");
	printf("----------------------------------------\r\n");
	printf("input: ");
}
int main(int argc, char **argv)
{
	int main_operate_code = 0;
	int temp32;
	bool valid_flag = false;
	bool err_flag = false;
	bool finished_before_timeout;

	ros::init(argc, argv, "sdk_wp_client");

	ros::NodeHandle n;

	ros::ServiceClient drone_control_manager = n.serviceClient<dji_ros::control_manager>("DJI_ROS/obtain_release_control");
	ros::ServiceClient drone_action_client = n.serviceClient<dji_ros::action>("DJI_ROS/drone_action_control");

	actionlib::SimpleActionClient<dji_ros::local_navigationAction> local_navigation_action_client("DJI_ROS/local_navigation_action", true);
	actionlib::SimpleActionClient<dji_ros::gps_navigationAction> gps_navigation_action_client("DJI_ROS/gps_navigation_action", true);
	actionlib::SimpleActionClient<dji_ros::waypoint_navigationAction> waypoint_navigation_action_client("DJI_ROS/waypoint_navigation_action", true);

	dji_ros::control_manager 	srv_control;
	dji_ros::action 				srv_action;
	dji_ros::local_navigationGoal goal_local;
	dji_ros::gps_navigationGoal goal_gps;
	dji_ros::waypoint_navigationGoal goal_waypoint;

	dji_ros::waypointList newWaypointList;
	dji_ros::waypoint waypoint0;
	dji_ros::waypoint waypoint1;
	dji_ros::waypoint waypoint2;
	dji_ros::waypoint waypoint3;
	dji_ros::waypoint waypoint4;

	Display_Main_Menu();
	while(1)
	{
		temp32 = getchar();
		if(temp32 != 10)
		{
			if(temp32 >= 'a' && temp32 <= 'i' && valid_flag == false)
			{
				main_operate_code = temp32;
				valid_flag = true;
			}
			else {
				err_flag = true;
			}
			continue;
		}
		else
		{
			if(err_flag == true)
			{
				printf("input: ERROR\n");
				Display_Main_Menu();
				err_flag = valid_flag = false;
				continue;
			}
		}
		switch(main_operate_code)
		{
			case 'a':
				/* request control ability*/
				srv_control.request.control_ability=1;
				drone_control_manager.call(srv_control);
				break;
			case 'b':
				/* release control ability*/
				srv_control.request.control_ability=0;
				drone_control_manager.call(srv_control);
				break;
			case 'c':
				/* take off */
				srv_action.request.action=4;
				drone_action_client.call(srv_action);
				break;
			case 'd':
				/* landing*/
				srv_action.request.action=6;
				drone_action_client.call(srv_action);
				break;
			case 'e':
				/* go home*/
				srv_action.request.action=1;
				drone_action_client.call(srv_action);
				break;
			case 'f':
				local_navigation_action_client.waitForServer();
				goal_local.x = -100;
				goal_local.y = -100;
				goal_local.z = 100;
				local_navigation_action_client.sendGoal(goal_local);

				finished_before_timeout = local_navigation_action_client.waitForResult(ros::Duration(300.0));

				if (finished_before_timeout)
				{
					actionlib::SimpleClientGoalState state = local_navigation_action_client.getState();
					ROS_INFO("Action finished: %s",state.toString().c_str());
				}
				else {
					ROS_INFO("Action did not finish before the time out.");
				}

				break;
			case 'g':
				gps_navigation_action_client.waitForServer();
				goal_gps.latitude = 22.535;
				goal_gps.longitude = 113.95;
				goal_gps.altitude = 100;
				gps_navigation_action_client.sendGoal(goal_gps);

				finished_before_timeout = gps_navigation_action_client.waitForResult(ros::Duration(300.0));

				if (finished_before_timeout)
				{
					actionlib::SimpleClientGoalState state = gps_navigation_action_client.getState();
					ROS_INFO("Action finished: %s",state.toString().c_str());
				}
				else {
					ROS_INFO("Action did not finish before the time out.");
				}

				break;

			case 'h':
				{
					waypoint0.latitude = 22.535;
					waypoint0.longitude = 113.95;
					waypoint0.altitude = 100;
					waypoint0.staytime = 5;
					waypoint0.heading = 0;
				}
				newWaypointList.waypointList.push_back(waypoint0);

				{
					waypoint1.latitude = 22.535;
					waypoint1.longitude = 113.96;
					waypoint1.altitude = 100;
					waypoint1.staytime = 0;
					waypoint1.heading = 90;
				}
				newWaypointList.waypointList.push_back(waypoint1);

				{
					waypoint2.latitude = 22.545;
					waypoint2.longitude = 113.96;
					waypoint2.altitude = 100;
					waypoint2.staytime = 4;
					waypoint2.heading = -90;
				}
				newWaypointList.waypointList.push_back(waypoint2);

				{
					waypoint3.latitude = 22.545;
					waypoint3.longitude = 113.96;
					waypoint3.altitude = 10;
					waypoint3.staytime = 2;
					waypoint3.heading = 180;
				}
				newWaypointList.waypointList.push_back(waypoint3);

				{
					waypoint4.latitude = 22.525;
					waypoint4.longitude = 113.93;
					waypoint4.altitude = 50;
					waypoint4.staytime = 0;
					waypoint4.heading = -180;
				}
				newWaypointList.waypointList.push_back(waypoint4);

				waypoint_navigation_action_client.waitForServer();
				goal_waypoint.waypointList = newWaypointList;
				waypoint_navigation_action_client.sendGoal(goal_waypoint);

				finished_before_timeout = waypoint_navigation_action_client.waitForResult(ros::Duration(300.0));

				if (finished_before_timeout)
				{
					actionlib::SimpleClientGoalState state = waypoint_navigation_action_client.getState();
					ROS_INFO("Action finished: %s",state.toString().c_str());
				}
				else {
					ROS_INFO("Action did not finish before the time out.");
				}

				break;

			case 'i':
				return 0;
				break;

			default:
				break;
		}
		main_operate_code = -1;
		err_flag = valid_flag = false;
		Display_Main_Menu();
	}
	return 0;
}
