#ifndef __DJI_SDK_LOCALS_H__
#define __DJI_SDK_LOCALS_H__
#include <dji_ros/global_position.h>
#include <dji_ros/local_position.h>
#include <dji_ros/attitude_quad.h>
#include <dji_ros/global_position.h>
#include <dji_ros/local_position.h>
#include <dji_ros/velocity.h>
#include <dji_ros/acc.h>
#include <dji_ros/gimbal.h>
#include <dji_ros/rc_channels.h>
#include <nav_msgs/Odometry.h>
#include "SDK.h"
#include "std_msgs/UInt8.h"

namespace dji_variable
{
	extern dji_ros::local_position local_position_ref;
	extern dji_ros::global_position global_position_ref;
	extern bool localposbase_use_height;
	extern dji_ros::attitude_quad attitude_quad;
	extern dji_ros::velocity velocity;
	extern dji_ros::acc acc;
	extern dji_ros::rc_channels rc_channels;
	extern dji_ros::global_position global_position;
	extern dji_ros::global_position global_position_degree;
	extern dji_ros::local_position local_position;
	extern dji_ros::compass compass_info;
	extern dji_ros::gimbal gimbal_info;
	extern float battery;
	extern uint8_t flight_status;
	extern dji_ros::ctrl_info ctrl_info;
	extern nav_msgs::Odometry odem;
	extern bool opened;
	extern bool activated;
	void gps_convert_ned(float &ned_x, float &ned_y,
			double gps_t_lon,
			double gps_t_lat,
			double gps_r_lon,
			double gps_r_lat
			);
	dji_ros::local_position gps_convert_ned(dji_ros::global_position loc);
};
#endif 
