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
			printf("Response Control");
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

	ros::ServiceServer control_service, action_service, camera_service, gimbal_angle_service, gimbal_speed_service, attitude_service;
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

		ROS_INFO("Init services\n");
		return 0;
	}
}
