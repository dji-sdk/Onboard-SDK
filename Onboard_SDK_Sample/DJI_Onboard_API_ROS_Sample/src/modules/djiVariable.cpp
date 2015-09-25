#include "djiVariable.h"
#include <dji_ros/attitude_quad.h>
#include <dji_ros/velocity.h>
#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793
namespace dji_variable
{
	dji_ros::local_position local_position_ref;
	dji_ros::global_position global_position_ref;
	bool localposbase_use_height = true;
	dji_ros::attitude_quad attitude_quad;
	dji_ros::velocity velocity;
	dji_ros::acc acc;
	dji_ros::rc_channels rc_channels;
	dji_ros::global_position global_position;
	dji_ros::global_position global_position_degree;
	dji_ros::local_position local_position;
	dji_ros::compass compass_info;
	dji_ros::gimbal gimbal_info;
	nav_msgs::Odometry odem;
	uint8_t flight_status;
	dji_ros::ctrl_info ctrl_info;
	float battery = 0;
	bool opened = false;
	bool activated = false;
	void gps_convert_ned(float &ned_x, float &ned_y,
			double gps_t_lon,
			double gps_t_lat,
			double gps_r_lon,
			double gps_r_lat
			)
	{
		//TODO :
		//fix bug with ellipsoid
		double d_lon = gps_t_lon - gps_r_lon;
		double d_lat = gps_t_lat - gps_r_lat;
		ned_x = d_lat * C_EARTH;
		ned_y = d_lon * C_EARTH * cos((gps_r_lat + gps_t_lat) / 2 * M_PI / 180.0f);
		return;
	}
	dji_ros::local_position gps_convert_ned(dji_ros::global_position loc)
	{
		dji_ros::local_position local;
		gps_convert_ned(
				local.x,
				local.y,
				loc.longitude,
				loc.latitude,
				global_position_ref.longitude,
				global_position_ref.latitude
				);
		local.height = global_position.height;
		return local;
	}
};
