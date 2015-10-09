#include "djiAction.h"
#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793

namespace action_handler
{
	
	task_action_type* task_action_ptr;
   local_navigation_action_type* local_navigation_action_ptr;
   gps_navigation_action_type* gps_navigation_action_ptr;
	waypoint_navigation_action_type* waypoint_navigation_action_ptr; 
	

	dji_ros::taskFeedback  task_action_feedback;
	dji_ros::taskResult  task_action_result; 

	dji_ros::local_navigationFeedback local_navigation_feedback; 
	dji_ros::local_navigationResult local_navigation_result; 

	dji_ros::gps_navigationFeedback gps_navigation_feedback;
	dji_ros::gps_navigationResult gps_navigation_result; 

	dji_ros::waypoint_navigationFeedback waypoint_navigation_feedback;
	dji_ros::waypoint_navigationResult waypoint_navigation_result;


	bool task_action_callback(const dji_ros::taskGoalConstPtr& goal, task_action_type* task_action)
	{
		uint8_t request_action = goal->task;

		if(request_action == 1)
		{
			//takeoff
			DJI_Pro_Status_Ctrl(4,0);
		}
		else if(request_action == 2)
		{
			//landing
			DJI_Pro_Status_Ctrl(6,0);
		}
		else if(request_action == 3)
		{
			//gohome
			DJI_Pro_Status_Ctrl(1,0);
		}

		task_action_feedback.progress = 1;
		task_action_ptr->publishFeedback(task_action_feedback);
		task_action_ptr->setSucceeded();
		

		return true;

	}

	bool local_navigation_action_callback(const dji_ros::local_navigationGoalConstPtr& goal, local_navigation_action_type* local_navigation_action)
	{
		/*IMPORTANT*/
		/*
			There has been declared a pointer `local_navigation_action` as the function parameter,
			However, it is the `local_navigation_action_ptr` that we should use.
			If `local_navigation_action` is used instead, there will be a runtime sengmentation fault.

			so interesting
		*/

		float dst_x = goal->x;
		float dst_y = goal->y;
		float dst_z = goal->z;

		float org_x = dji_variable::local_position.x;
		float org_y = dji_variable::local_position.y;
		float org_z = dji_variable::local_position.z;

		float dis_x = dst_x - org_x;
		float dis_y = dst_y - org_y;
		float dis_z = dst_z - org_z; 

		float det_x,det_y,det_z;

		attitude_data_t user_ctrl_data;
      user_ctrl_data.ctrl_flag = 0x90;
      user_ctrl_data.thr_z = dst_z;
      user_ctrl_data.yaw = 0;

		int x_progress = 0; 
		int y_progress = 0; 
		int z_progress = 0; 
		while (x_progress < 100 || y_progress < 100 || z_progress <100) {

         user_ctrl_data.roll_or_x = dst_x - dji_variable::local_position.x;
         user_ctrl_data.pitch_or_y = dst_y - dji_variable::local_position.y;

         DJI_Pro_Attitude_Control(&user_ctrl_data);

			det_x = (100* (dst_x - dji_variable::local_position.x))/dis_x;
			det_y = (100* (dst_y - dji_variable::local_position.y))/dis_y;
			det_z = (100* (dst_z - dji_variable::local_position.z))/dis_z;

			x_progress = 100 - (int)det_x;
			y_progress = 100 - (int)det_y;
			z_progress = 100 - (int)det_z;

			local_navigation_feedback.x_progress = x_progress;
			local_navigation_feedback.y_progress = y_progress;
			local_navigation_feedback.z_progress = z_progress;
			local_navigation_action_ptr->publishFeedback(local_navigation_feedback);

         usleep(20000);

      }

		local_navigation_result.result = true;
		local_navigation_action_ptr->setSucceeded(local_navigation_result);

		return true;
	}

	bool gps_navigation_action_callback(const dji_ros::gps_navigationGoalConstPtr& goal, gps_navigation_action_type* gps_navigation_action)
	{
		//The GPS coordiante sendto/receivefrom M100 are both in radian.
		double dst_latitude = goal->latitude*C_PI/180;
		double dst_longitude = goal->longitude*C_PI/180;
		float dst_altitude = goal->altitude;
		
		double org_latitude = dji_variable::global_position.latitude;
		double org_longitude = dji_variable::global_position.longitude;
		float org_altitude = dji_variable::global_position.altitude;

		double dis_x,dis_y;
		float dis_z;
		
		dis_x = dst_latitude - org_latitude;
		dis_y = dst_longitude - org_longitude;
		dis_z = dst_altitude - org_altitude;

		double det_x,det_y;
		float det_z;

		attitude_data_t user_ctrl_data;
      user_ctrl_data.ctrl_flag = 0x90;
      user_ctrl_data.thr_z = dst_altitude;
      user_ctrl_data.yaw = 0;
	
		int latitude_progress = 0; 
		int longitude_progress = 0; 
		int altitude_progress = 0; 

		while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress <100) {

			user_ctrl_data.roll_or_x = (dst_latitude - dji_variable::global_position.latitude)*C_EARTH;
			user_ctrl_data.pitch_or_y = (dst_longitude - dji_variable::global_position.longitude)*C_EARTH*cos(dji_variable::global_position.latitude);

         DJI_Pro_Attitude_Control(&user_ctrl_data);

			det_x = (100* (dst_latitude - dji_variable::global_position.latitude))/dis_x;
			det_y = (100* (dst_longitude - dji_variable::global_position.longitude))/dis_y;
			det_z = (100* (dst_altitude - dji_variable::global_position.altitude))/dis_z;
		

			latitude_progress = 100 - (int)det_x;
			longitude_progress = 100 - (int)det_y;
			altitude_progress = 100 - (int)det_z;

			//lazy evaluation
			if (abs(dis_x) < 0.00001) latitude_progress = 100;
			if (abs(dis_y) < 0.00001) longitude_progress = 100;
			if (abs(dis_z) < 1) altitude_progress = 100;

			gps_navigation_feedback.latitude_progress = latitude_progress;
			gps_navigation_feedback.longitude_progress = longitude_progress;
			gps_navigation_feedback.altitude_progress = altitude_progress;
			gps_navigation_action_ptr->publishFeedback(gps_navigation_feedback);

         usleep(20000);

      }

		gps_navigation_result.result = true;
		gps_navigation_action_ptr->setSucceeded(gps_navigation_result);

		return true;
	}

	bool waypoint_navigation_action_callback(const dji_ros::waypoint_navigationGoalConstPtr& goal, waypoint_navigation_action_type* waypoint_navigation_action)
	{
		dji_ros::waypointList newWaypointList;
		newWaypointList = goal->waypointList;

		for (int i = 0; i < newWaypointList.waypointList.size(); i++) {
			const dji_ros::waypoint newWaypoint = newWaypointList.waypointList[i];	
			waypoint_navigation_feedback.index_progress = i;
			processWaypoint(newWaypoint);
		}
		
		waypoint_navigation_result.result = true;
		waypoint_navigation_action_ptr->setSucceeded(waypoint_navigation_result);

		return true;
	}

	void processWaypoint(dji_ros::waypoint newWaypoint) {

		double dst_latitude = newWaypoint.latitude*C_PI/180;
		double dst_longitude = newWaypoint.longitude*C_PI/180;
		float dst_altitude = newWaypoint.altitude;

		double org_latitude = dji_variable::global_position.latitude;
		double org_longitude = dji_variable::global_position.longitude;
		float org_altitude = dji_variable::global_position.altitude;

		double dis_x,dis_y;
		float dis_z;

		dis_x = dst_latitude - org_latitude;
		dis_y = dst_longitude - org_longitude;
		dis_z = dst_altitude - org_altitude;

		double det_x,det_y;
		float det_z;

		attitude_data_t user_ctrl_data;
      user_ctrl_data.ctrl_flag = 0x90;
      user_ctrl_data.thr_z = dst_altitude;
      user_ctrl_data.yaw = newWaypoint.heading;
	
		int latitude_progress = 0; 
		int longitude_progress = 0; 
		int altitude_progress = 0; 

		while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress <100) {

			user_ctrl_data.roll_or_x = (dst_latitude - dji_variable::global_position.latitude)*C_EARTH;
			user_ctrl_data.pitch_or_y = (dst_longitude - dji_variable::global_position.longitude)*C_EARTH*cos(dji_variable::global_position.latitude);

         DJI_Pro_Attitude_Control(&user_ctrl_data);

			det_x = (100* (dst_latitude - dji_variable::global_position.latitude))/dis_x;
			det_y = (100* (dst_longitude - dji_variable::global_position.longitude))/dis_y;
			det_z = (100* (dst_altitude - dji_variable::global_position.altitude))/dis_z;
		

			latitude_progress = 100 - abs((int)det_x);
			longitude_progress = 100 - abs((int)det_y);
			altitude_progress = 100 - abs((int)det_z);

			//lazy evaluation when moving distance tooooooooooo small
			//need to find a better way
			if (fabs((dst_latitude - dji_variable::global_position.latitude)*180/C_PI) < 0.00001) latitude_progress = 100;
			if (fabs((dst_longitude - dji_variable::global_position.longitude)*180/C_PI) < 0.00001) longitude_progress = 100;
			if (fabsf(dst_altitude - dji_variable::global_position.altitude) < 0.1) altitude_progress = 100;

			waypoint_navigation_feedback.latitude_progress = latitude_progress;
			waypoint_navigation_feedback.longitude_progress = longitude_progress;
			waypoint_navigation_feedback.altitude_progress = altitude_progress;
			waypoint_navigation_action_ptr->publishFeedback(waypoint_navigation_feedback);

         usleep(20000);

      }
		ros::Duration(newWaypoint.staytime).sleep();

	}

	int init_actions(ros::NodeHandle &n)
	{
		task_action_ptr = new task_action_type(n, "DJI_ROS/task_action",boost::bind(&task_action_callback, _1, task_action_ptr), false);
		//task_action_ptr->start();

		local_navigation_action_ptr = new local_navigation_action_type(n,"DJI_ROS/local_navigation_action", boost::bind(&local_navigation_action_callback, _1, local_navigation_action_ptr), false );
		local_navigation_action_ptr->start();

		gps_navigation_action_ptr = new gps_navigation_action_type(n,"DJI_ROS/gps_navigation_action", boost::bind(&gps_navigation_action_callback, _1, gps_navigation_action_ptr), false );
		gps_navigation_action_ptr->start();

		waypoint_navigation_action_ptr = new waypoint_navigation_action_type(n,"DJI_ROS/waypoint_navigation_action", boost::bind(&waypoint_navigation_action_callback, _1, waypoint_navigation_action_ptr), false );
		waypoint_navigation_action_ptr->start();

		return 0;
	}
}
