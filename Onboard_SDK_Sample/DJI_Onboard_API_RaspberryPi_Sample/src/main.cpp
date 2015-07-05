//============================================================================
// Name        : DJI_Onboard_Sdk_Test.cpp
// Author      : wuyuwei
// Modified By : Evan.Gu
// Version     :
// Copyright   : DJI Inc
// Description : DJI Onboard SDK test in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <string.h>
#include "DJI_Pro_Test.h"
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
	printf("[g]  Query UAV current status\n");

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



	if(argc == 2 && strcmp(argv[1],"-v") == 0)
	{
		printf("\nDJI Onboard API Cmdline Test,Ver 1.0.0\n\n");
		return 0;
	}
	printf("\nDJI Onboard API Cmdline Test,Ver 1.0.0\n\n");

	if(DJI_Pro_Test_Setup() < 0)
	{
		printf("Serial Port Open ERROR\n");
                printf("Permission denied? Try \"sudo chmod 777 /dev/ttyUSB0\"\n");
		return 0;
	}
	Display_Main_Menu();
	while(1)
	{
		temp32 = getchar();
		if(temp32 != 10)
		{
			if(temp32 >= 'a' && temp32 <= 'g' && valid_flag == false)
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
			DJI_Onboard_API_Activation();
			break;
		case 'b':
			/* get controller */
			DJI_Onboard_API_Control(1);
			break;
		case 'c':
			/* release controller */
			DJI_Onboard_API_Control(0);
			break;
		case 'd':
			/* takeoff */
			DJI_Onboard_API_UAV_Control(4);
			break;
		case 'e':
			/* landing */
			DJI_Onboard_API_UAV_Control(6);
			break;
		case 'f':
			/* go home */
			DJI_Onboard_API_UAV_Control(1);
			break;
		case 'g':
			/* status query */
			DJI_Onboard_API_Status_Query();
			break;
		}
		main_operate_code = -1;
		err_flag = valid_flag = false;
		Display_Main_Menu();
	}

	return 0;
}
