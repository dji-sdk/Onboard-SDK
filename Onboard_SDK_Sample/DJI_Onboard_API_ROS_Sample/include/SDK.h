#ifndef SDK_LIBRARY_H
#define SDK_LIBRARY_H
//SDK library
#include "DJI_LIB/DJI_Pro_App.h" 
#include "DJI_LIB/DJI_Pro_Codec.h" 
#include "DJI_LIB/DJI_Pro_Config.h" 
#include "DJI_LIB/DJI_Pro_Hw.h" 
#include "DJI_LIB/DJI_Pro_Link.h" 
#include "DJI_LIB/DJI_Pro_Rmu.h"

//msgs
#include <dji_ros/acc.h>
#include <dji_ros/attitude_quad.h>
#include <dji_ros/gimbal.h>
#include <dji_ros/global_position.h>
#include <dji_ros/local_position.h>
#include <dji_ros/rc_channels.h>
#include <dji_ros/velocity.h>
#include <dji_ros/compass.h>
#include <dji_ros/ctrl_info.h>
#include <dji_ros/waypoint.h>
#include <dji_ros/waypointList.h>
#include <dji_ros/map_nav_srv_cmd.h>

//srvs
#include <dji_ros/attitude.h>
#include <dji_ros/action.h>
#include <dji_ros/camera_action.h>
#include <dji_ros/control_manager.h>
#include <dji_ros/gimbal_speed.h>
#include <dji_ros/gimbal_angle.h>

//action
#include <dji_ros/taskAction.h>
#include <dji_ros/local_navigationAction.h>
#include <dji_ros/gps_navigationAction.h>
#include <dji_ros/waypoint_navigationAction.h>
#include <dji_ros/web_waypoint_receiveAction.h>
#endif
