#ifndef __DJI_SDK_NODE_H__
#define __DJI_SDK_NODE_H__
#include <ros/ros.h>
#include "SDK.h"
#include "djiVariable.h"
#include "djiPublisher.h"
#include "djiService.h"

std::string serial_name;
int	baud_rate;
int	app_id;
int 	app_api_level;
int	app_version;
std::string app_bundle_id;
std::string enc_key;

static activate_data_t user_act_data;
static sdk_std_msg_t recv_sdk_std_msgs;
static req_id_t nav_force_close_req_id;


void update_ros_vars();
int DJI_Setup(std::string, int );
#endif
