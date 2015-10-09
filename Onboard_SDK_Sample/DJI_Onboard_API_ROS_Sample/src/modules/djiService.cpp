#include "djiService.h"

namespace service_handler
{
    bool control_callback(
			dji_ros::control_manager::Request& request,
			dji_ros::control_manager::Response& response
			)
	{
		if (request.control_ability== 1) {
			printf("Request Control");
            DJI_Pro_Control_Management(1,NULL);
            response.result= true;
		}
		else if (request.control_ability== 0) {
			printf("Release Control");
			DJI_Pro_Control_Management(0,NULL);
			response.result = true;
		}
		else
			response.result = false;

		return true;
	}

	bool action_callback(
			dji_ros::action::Request& request,
			dji_ros::action::Response& response
			)
	{
		if(request.action== 4)
		{
			//takeoff
			DJI_Pro_Status_Ctrl(4,0);
			response.result= true;
		}
		else if(request.action == 6)
		{
			//landing
			DJI_Pro_Status_Ctrl(6,0);
			response.result= true;
		}
		else if(request.action == 1)
		{
			//gohome
			DJI_Pro_Status_Ctrl(1,0);
			response.result= true;
		}
		else
			response.result= false;
		return true;
	}

	bool camera_action_callback(
			dji_ros::camera_action::Request& request,
			dji_ros::camera_action::Response& response
			)
	{
		if (request.camera_action == 0){
			DJI_Pro_Camera_Control(API_CAMERA_SHOT);
			response.result = true;
		}
		else if (request.camera_action == 1){
			DJI_Pro_Camera_Control(API_CAMERA_VIDEO_START);
			response.result = true;
		}
		else if (request.camera_action == 2){
			DJI_Pro_Camera_Control(API_CAMERA_VIDEO_STOP);
			response.result = true;
		}
		else
			response.result = false;
		return true;

	}
		

	bool gimbal_angle_callback(
			dji_ros::gimbal_angle::Request& request,
			dji_ros::gimbal_angle::Response& response
			)
	{
		uint8_t flag =request.flag;
		int16_t x = request.x;
		int16_t y = request.y;
		int16_t yaw = request.yaw;
		uint8_t duration = request.duration;

		DJI_Sample_Gimbal_AngleCtrl(yaw, x, y, flag, duration);
		response.result = true;
		return true;
	}

	bool gimbal_speed_callback(
			dji_ros::gimbal_speed::Request& request,
			dji_ros::gimbal_speed::Response& response
			)
	{
		signed short yaw_rate = request.yaw_rate;
		signed short x_rate = request.x_rate;
		signed short y_rate = request.y_rate;

		DJI_Sample_Gimbal_SpeedCtrl(yaw_rate, x_rate, y_rate);

		response.result = true;
		return true;
	}

	bool attitude_callback(
			dji_ros::attitude::Request& request,
			dji_ros::attitude::Response& response
			)
	{
		attitude_data_t user_ctrl_data;

		user_ctrl_data.ctrl_flag = request.flag;
		user_ctrl_data.roll_or_x = request.x;
		user_ctrl_data.pitch_or_y = request.y;
		user_ctrl_data.thr_z = request.z;
		user_ctrl_data.yaw = request.yaw;

		DJI_Pro_Attitude_Control(&user_ctrl_data);

		response.result = true;
		return true;

	}

/*
	bool local_navigation_callback(
			dji_ros::local_navigation::Request& request,
			dji_ros::local_navigation::Response& response
			)
	{
		//actually, this is the same as attitude service, but with HORI_POS and VERT_POS in ground frame i.e. 0b1001x00x
		float dst_x = request.x;
		float dst_y = request.y;
		float dst_z = request.z;

		attitude_data_t user_ctrl_data;
		user_ctrl_data.ctrl_flag = 0x90;
		user_ctrl_data.thr_z = dst_z;
		user_ctrl_data.yaw = 0;


		while (sqrt(pow(dst_x - dji_variable::local_position.x,2) + pow(dst_y - dji_variable::local_position.y,2) + pow(dst_z - dji_variable::local_position.height,2)) > 0.5) {

			user_ctrl_data.roll_or_x = dst_x - dji_variable::local_position.x;
			user_ctrl_data.pitch_or_y = dst_y - dji_variable::local_position.y;

			DJI_Pro_Attitude_Control(&user_ctrl_data);
			usleep(20000);

		}

		response.result = true;
		return true;
	}

	bool gps_navigation_callback(
			dji_ros::gps_navigation::Request& request,
			dji_ros::gps_navigation::Response& response
			)
	{
		//after convert det(GPS) into distance, this is the same as local_navigation
			
		return true;
	}


	bool waypoints_callback(
			dji_ros::waypoints_navigation::Request& request,
			dji_ros::waypoints_navigation::Response& response
			)
	{
		//waypointList[] waypoints;
		//an example(not sure correct or not) of using an msg, which contains an array of another custom msgs as a custom srv request
		 *
		 *for (int i = 0; i < waypointList -> size; i++):
		 *		dji_ros::waypoint wpData = waypointList -> waypoint [i]
		 *
		//separate out each waypoint, then work is the same as gps_navigation 
		//however, extra functions are necessary to handle stay time 
		//as for yaw, 0x90 has already been the YAW_ANG mode(0bxxxx0xx0), just set (-180,180) is okay

		return true;
	}
*/

	ros::ServiceServer control_service, camera_service, gimbal_angle_service, gimbal_speed_service, attitude_service, action_service;
	int init_services(ros::NodeHandle & n)
	{
		control_service = n.advertiseService(
				"DJI_ROS/obtain_release_control",
				control_callback
				);

		action_service =n.advertiseService(
				"DJI_ROS/drone_action_control",
				action_callback
				);

		camera_service =n.advertiseService(
				"DJI_ROS/camera_action_service",
				camera_action_callback
				);

		gimbal_angle_service = n.advertiseService(
				"DJI_ROS/gimbal_angle_control",
				gimbal_angle_callback
				);

		gimbal_speed_service = n.advertiseService(
				"DJI_ROS/gimbal_speed_control",
				gimbal_speed_callback
				);

		attitude_service = n.advertiseService(
				"DJI_ROS/drone_attitude_control",
				attitude_callback
				);

		/*
		local_navigation_service = n.advertiseService(
				"DJI_ROS/local_navigation_service",
				local_navigation_callback
				);

		gps_navigation_service = n.advertiseService(
				"DJI_ROS/gps_navigation_service",
				gps_navigation_callback
				);
		
		waypoints_navigation_service = n.advertiseService(
				"DJI_ROS/waypoints_service",
				waypoints_callback
				);
		*/

		
		return 0;
	}
}
