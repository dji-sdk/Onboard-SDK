#ifndef DJI_SDK_DJI_SERVICES_H
#define DJI_SDK_DJI_SERVICES_H
#include <ros/ros.h>
#include "SDK.h"
#include "djiVariable.h"

namespace service_handler
{
	int init_services(ros::NodeHandle & n);

	static void DJI_Sample_Gimbal_SpeedCtrl(signed short yaw_angle_rate,
			signed short roll_angle_rate,
			signed short pitch_angle_rate)
	{
		gimbal_custom_speed_t gimbal_speed = {0};
		gimbal_speed.yaw_angle_rate = yaw_angle_rate;
		gimbal_speed.roll_angle_rate = roll_angle_rate;
		gimbal_speed.pitch_angle_rate = pitch_angle_rate;
		gimbal_speed.ctrl_byte.ctrl_switch = 1;
		DJI_Pro_App_Send_Data(0,
				1, MY_CTRL_CMD_SET,
				API_GIMBAL_CTRL_SPEED_REQUEST,
				(uint8_t*)&gimbal_speed,
				sizeof(gimbal_speed),
				NULL,
				0,
				0
				);
	}
	static void DJI_Sample_Gimbal_AngleCtrl(int16_t yaw_angle,
			int16_t roll_angle,
			int16_t pitch_angle,
			uint8_t baseflag,
			uint8_t duration)
	{
		gimbal_custom_control_angle_t gimbal_angle = {0};
		gimbal_angle.yaw_angle = yaw_angle;
		gimbal_angle.roll_angle = roll_angle;
		gimbal_angle.pitch_angle = pitch_angle;
		gimbal_angle.ctrl_byte.base = baseflag;
		gimbal_angle.ctrl_byte.yaw_cmd_ignore = 0;
		gimbal_angle.ctrl_byte.roll_cmd_ignore = 0;
		gimbal_angle.ctrl_byte.pitch_cmd_ignore = 0;
		gimbal_angle.duration = duration;
		DJI_Pro_App_Send_Data(0,
				0,
				MY_CTRL_CMD_SET,
				API_GIMBAL_CTRL_ANGLE_REQUEST,
				(uint8_t*)&gimbal_angle,
				sizeof(gimbal_angle),
				NULL,
				0,
				0
				);
	}

}
#endif 
