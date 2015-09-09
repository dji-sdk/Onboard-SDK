//============================================================================
// Name        : main.cpp
// Author      : wuyuwei
// Version     :
// Copyright   : DJI Inc
// Description : DJI Onboard API test in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <string.h>
#include "DJI_Pro_Sample.h"

using namespace std;

static void Display_Main_Menu(void)
{
	printf("\r\n");
	printf("-----------   < Main menu >  ----------\r\n\r\n");

	printf("[a]  Request activation\n");
	printf("[b]  Request to obtain control\n");
	printf("[c]  Release control\n");
	printf("[d]  Takeoff\n");
	printf("[e]  Landing\n");
	printf("[f]  Go home\n");
	printf("[g]  Attitude control sample\n");
	printf("[h]  Gimbal control sample\n");
	printf("[i]  Query UAV current status\n");

	printf("\ninput a/b/c etc..then press enter key\r\n");
	printf("---------------------------------------\r\n");
	printf("input: ");
}

int main(int argc,char **argv)
{
	int main_operate_code = 0;
	int temp32;
	bool valid_flag = false;
	bool err_flag = false;
	activate_data_t user_act_data; 
	char temp_buf[65];

	if(argc == 2 && strcmp(argv[1],"-v") == 0)
	{
		printf("\nDJI Onboard API Cmdline Test,Ver 1.0.0\n\n");
		return 0;
	}
	printf("\nDJI Onboard API Cmdline Test,Ver 1.1.0\n\n");

	if(DJI_Sample_Setup() < 0)
	{
		printf("Serial Port Open ERROR\n");
		return 0;
	}

	user_act_data.app_key = temp_buf;
	user_act_data.app_ver = SDK_VERSION;
    	if(DJI_Pro_Get_Cfg(NULL,NULL,&user_act_data.app_id,&user_act_data.app_api_level,user_act_data.app_key) == 0)
	{
		/* user setting */
		printf("--------------------------\n");
		printf("app id=%d\n",user_act_data.app_id);
		printf("app api level=%d\n",user_act_data.app_api_level);
		printf("app key=%s\n",user_act_data.app_key);
		printf("--------------------------\n");
	}
	else
	{
		printf("ERROR:There is no user account\n");	
		return 0;
	}

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
			else
			{
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
			/* api activation */
			DJI_Pro_Activate_API(&user_act_data,NULL);
			break;
		case 'b':
			/* get controller */
			DJI_Pro_Control_Management(1,NULL);
			break;
		case 'c':
			/* release controller */
			DJI_Pro_Control_Management(0,NULL);
			break;
		case 'd':
			/* takeoff */
			DJI_Pro_Status_Ctrl(4,0);
			break;
		case 'e':
			/* landing */
			DJI_Pro_Status_Ctrl(6,0);
			break;
		case 'f':
			/* go home */
			DJI_Pro_Status_Ctrl(1,0);
			break;
		case 'g':
			/* attitude ctrl */
			if(DJI_Sample_Atti_Ctrl()< 0)
    			{
        			printf("Please waiting current sample finish\n");
   			}
			break;
		case 'h':
			if(DJI_Sample_Gimbal_Ctrl()< 0)
    			{
        			printf("Please waiting current sample finish\n");
   			}
			break;
		case 'i':
			/* status query */
			DJI_Sample_Drone_Status_Query();
			break;
		}
		main_operate_code = -1;
		err_flag = valid_flag = false;
		Display_Main_Menu();
	}

	return 0;
}
