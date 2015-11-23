/*
 * DJI_Pro_Sample.cpp
 *
 *  Created on: Aug 24, 2015
 *  Author: wuyuwei
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include "DJI_Pro_Sample.h"

#if (defined(PLATFORM_LINUX) && defined(TINYXML_CFG))
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "tinyxml2.h"
using namespace tinyxml2;
using namespace std;
#endif

static int atti_ctrl_sample_flag = -1;
/*
 * attitude control sample
 */
static int DJI_Sample_Create_Thread(void *(* func)(void *), void *arg)
{
    pthread_t A_ARR;

    if(pthread_create(&A_ARR,0,func,arg) != 0)
    {
        return -1;
    }
    return 0;
}

/*
 * This method is responsible for testing atttitude
 */
static void * DJI_Sample_Atti_Ctrl_Thread_Func(void *arg)
{
    int i;
    attitude_data_t user_ctrl_data;

    /* takeoff */
    DJI_Pro_Status_Ctrl(4,0);
    sleep(8);
    /* attitude control, go up */
    for(i = 0; i < 100; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        if(i < 90)
            user_ctrl_data.thr_z = 2.0;
        else
            user_ctrl_data.thr_z = 0.0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        if(i < 180)
            user_ctrl_data.roll_or_x = 2;
        else
            user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        if(i < 180)
            user_ctrl_data.roll_or_x = -2;
        else
            user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        user_ctrl_data.roll_or_x = 0;
        if(i < 180)
            user_ctrl_data.pitch_or_y = 2;
        else
            user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        user_ctrl_data.roll_or_x = 0;
        if(i < 180)
            user_ctrl_data.pitch_or_y = -2;
        else
            user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        if(i < 180)
            user_ctrl_data.thr_z = 0.5;
        else
            user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        if(i < 180)
            user_ctrl_data.thr_z = -0.5;
        else
            user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0xA;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        if(i < 180)
            user_ctrl_data.yaw = 90;
        else
            user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    for(i = 0; i < 200; i ++)
    {
        user_ctrl_data.ctrl_flag = 0xA;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        if(i < 180)
            user_ctrl_data.yaw = -90;
        else
            user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(1);
    /* gohome */
    DJI_Pro_Status_Ctrl(1,0);

    atti_ctrl_sample_flag = -1;
    return (void*)NULL;
}

int DJI_Sample_Atti_Ctrl(void)
{
    if(atti_ctrl_sample_flag == 0)
    {
        return -1;
    }
    atti_ctrl_sample_flag = 0;

    if(DJI_Sample_Create_Thread(DJI_Sample_Atti_Ctrl_Thread_Func,NULL) != 0)
    {
        return -1;
    }
    return 0;
}

/*
 * gimbal control sample
 */
static int gimbal_ctrl_sample_flag = -1;
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
                   1,
                   MY_CTRL_CMD_SET,
                   API_GIMBAL_CTRL_SPEED_REQUEST,
                   (uint8_t*)&gimbal_speed,
                   sizeof(gimbal_speed),
                   NULL,
                   0,
                   0
                   );
}

static void DJI_Sample_Gimbal_AngelCtrl(int16_t yaw_angle,
                       int16_t roll_angle,
                       int16_t pitch_angle,
                       uint8_t duration)
{
    gimbal_custom_control_angle_t gimbal_angel = {0};

    gimbal_angel.yaw_angle = yaw_angle;
    gimbal_angel.roll_angle = roll_angle;
    gimbal_angel.pitch_angle = pitch_angle;
    gimbal_angel.ctrl_byte.base = 0;
    gimbal_angel.ctrl_byte.yaw_cmd_ignore = 0;
    gimbal_angel.ctrl_byte.roll_cmd_ignore = 0;
    gimbal_angel.ctrl_byte.pitch_cmd_ignore = 0;
    gimbal_angel.duration = duration;

    DJI_Pro_App_Send_Data(0,
                  0,
                  MY_CTRL_CMD_SET,
                  API_GIMBAL_CTRL_ANGLE_REQUEST,
                  (uint8_t*)&gimbal_angel,
                  sizeof(gimbal_angel),
                  NULL,
                  0,
                  0
                  );

}

/*
 * This method is responsible for testing gimbal
 */
static void * DJI_Sample_Gimbal_Ctrl_Thread_Func(void * arg)
{
    int i = 0;
    arg = arg;

    printf("\nGimbal test start...\r\n");

    DJI_Sample_Gimbal_AngelCtrl(1800, 0, 0, 20);
    sleep(2);
    usleep(100000);
    DJI_Sample_Gimbal_AngelCtrl(-1800, 0, 0, 20);
    sleep(2);
    usleep(100000);
    DJI_Sample_Gimbal_AngelCtrl(0, 300, 0, 20);
    sleep(2);
    usleep(100000);
    DJI_Sample_Gimbal_AngelCtrl(0, -300, 0, 20);
    sleep(2);
    usleep(100000);
    DJI_Sample_Gimbal_AngelCtrl(0, 0, 300, 20);
    sleep(2);
    usleep(100000);
    DJI_Sample_Gimbal_AngelCtrl(0, 0, -300, 20);
    sleep(2);
    usleep(100000);

    for(i = 0; i < 20; i++)
    {
        DJI_Sample_Gimbal_SpeedCtrl(200, 0, 0);
        usleep(100000);
    }
    for(i = 0; i < 20; i++)
    {
        DJI_Sample_Gimbal_SpeedCtrl(-200, 0, 0);
        usleep(100000);
    }
    for(i = 0; i < 20; i++)
    {
        DJI_Sample_Gimbal_SpeedCtrl(0, 200, 0);
        usleep(100000);
    }
    for(i = 0; i < 20; i++)
    {
        DJI_Sample_Gimbal_SpeedCtrl(0, -200, 0);
        usleep(100000);
    }
    for(i = 0; i < 20; i++)
    {
        DJI_Sample_Gimbal_SpeedCtrl(0, 0, 200);
        usleep(100000);
    }
    for(i = 0; i < 20; i++)
    {
        DJI_Sample_Gimbal_SpeedCtrl(0, 0, -200);
        usleep(100000);
    }
    DJI_Sample_Gimbal_AngelCtrl(0, 0, 0, 10);
    gimbal_ctrl_sample_flag = -1;
    printf("Gimbal test end.\r\n");
    return (void*)NULL;
}

int DJI_Sample_Gimbal_Ctrl(void)
{
    if(gimbal_ctrl_sample_flag == 0)
    {
        return -1;
    }
    gimbal_ctrl_sample_flag = 0;

    if(DJI_Sample_Create_Thread(DJI_Sample_Gimbal_Ctrl_Thread_Func,NULL) != 0)
    {
        return -1;
    }
    return 0;
}

/*
 * This method is responsible for camera shotting
 */
void DJI_Sample_Camera_Shot()
{
    DJI_Pro_Camera_Control(API_CAMERA_SHOT);
}

/*
 *This method is responsible for starting camera video
 */
void DJI_Sample_Camera_Video_Start()
{
    DJI_Pro_Camera_Control(API_CAMERA_VIDEO_START);
}

/*
 * This method is responsible for stoping camera video
 */
void DJI_Sample_Camera_Video_Stop()
{
    DJI_Pro_Camera_Control(API_CAMERA_VIDEO_STOP);
}

/*
 *This method is responsible for sending data to mobile device
 */
void DJI_Sample_Send_To_Mobile_Device()
{
    unsigned char send_data[100] = "hello world!";
    DJI_Pro_Send_To_Mobile_Device(send_data,sizeof(send_data),NULL);
}

/*
 * This method is responsible for flying a circle controled by position
 */
void DJI_Sample_Circle_By_Pos()
{
    attitude_data_t user_ctrl_data;

    static float time = 0;
    static float R = 2;
    static float V = 2;
    static float vx;
    static float vy;
    int i;

    /* start to draw circle */
    for(i = 0; i < 300; i ++)
    {
        vx = V * sin((V/R)*time/50.0f);
        vy = V * cos((V/R)*time/50.0f);
        user_ctrl_data.ctrl_flag = HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY;
        user_ctrl_data.roll_or_x = vx;
        user_ctrl_data.pitch_or_y = vy;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
        time++;
    }
}

/*
 * This method is responsible for flying a square controled by position
 */
void DJI_Sample_Square_By_Pos()
{
    attitude_data_t user_ctrl_data;
    int i;
    for(i = 0;i < 60;i++)
    {
        user_ctrl_data.ctrl_flag = HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY;
        user_ctrl_data.roll_or_x = 3;
        user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    for(i = 0;i < 60;i++)
    {
        user_ctrl_data.ctrl_flag = HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 3;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    for(i = 0;i < 60;i++)
    {
        user_ctrl_data.ctrl_flag = HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY;
        user_ctrl_data.roll_or_x = -3;
        user_ctrl_data.pitch_or_y = 0;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    for(i = 0;i < 60;i++)
    {
        user_ctrl_data.ctrl_flag = HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = -3;
        user_ctrl_data.thr_z = 0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
}

void DJI_CalOffset(const fp64 Cur_lati_r, const fp64 Cur_longti_r, const fp32 Cur_alti_m, \
            fp64* const lati_offset_m, fp64* const longti_offset_m, fp32* const alti_offset_m,\
            const fp64 Tar_lati_r, const fp64 Tar_longti_r, const fp32 Tar_alti_m)
{
    *lati_offset_m = (Tar_lati_r - Cur_lati_r)*(double)6378137.0;
    *longti_offset_m = (Tar_longti_r - Cur_longti_r)*((double)6378137.0*cos(Cur_lati_r));
    *alti_offset_m = Tar_alti_m - Cur_alti_m;
}

void DJI_CalPos(const fp64 Cur_lati_r, const fp64 Cur_longti_r, const fp32 Cur_alti_m, \
            const fp64 lati_offset_m, const fp64 longti_offset_m, const fp32 alti_offset_m, \
            fp64* const Tar_lati_r, fp64* const Tar_longti_r, fp32* const Tar_alti_m)
{
    *Tar_lati_r = Cur_lati_r + (lati_offset_m/(double)6378137.0);
    *Tar_longti_r = Cur_longti_r + (longti_offset_m/((double)6378137.0*cos(Cur_lati_r)));
    *Tar_alti_m = Cur_alti_m + alti_offset_m;
}

void DJI_GotoPos(fp64 lati, fp64 longti, fp32 alti,fp32 err)
{
    sdk_std_msg_t UavInfo;
    attitude_data_t user_ctrl_data;
    fp64 lati_offset_m = 0;
    fp64 longti_offset_m = 0;
    fp32 alti_offset_m = 0;
    while(1)
    {
        DJI_Pro_Get_Broadcast_Data(&UavInfo);

        DJI_CalOffset(UavInfo.pos.lati,UavInfo.pos.longti,UavInfo.pos.alti,\
               &lati_offset_m,&longti_offset_m,&alti_offset_m,\
               lati,longti,alti);


        printf("lati = %f longti = %f alti = %f sqrt = %f\r",lati_offset_m,longti_offset_m,alti,\
               sqrt(lati_offset_m*lati_offset_m + longti_offset_m*longti_offset_m + alti_offset_m*alti_offset_m));

        if(sqrt(lati_offset_m*lati_offset_m + longti_offset_m*longti_offset_m + alti_offset_m*alti_offset_m) <= err)
        {
            user_ctrl_data.ctrl_flag = HORIZ_POS|VERT_VEL|YAW_RATE|HORIZ_BODY|YAW_BODY;
            user_ctrl_data.roll_or_x = 0;
            user_ctrl_data.pitch_or_y = 0;
            user_ctrl_data.thr_z = 0;
            user_ctrl_data.yaw = 0;
            DJI_Pro_Attitude_Control(&user_ctrl_data);
            break;
        }

        user_ctrl_data.ctrl_flag = HORIZ_POS|VERT_POS|YAW_ANG|HORIZ_BODY|YAW_BODY;
        user_ctrl_data.roll_or_x = lati_offset_m;
        user_ctrl_data.pitch_or_y = longti_offset_m;
        user_ctrl_data.thr_z = alti;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
}

void DJI_Sample_Way_Point()
{
    typedef struct way_point
    {
        fp64 lati_r;
        fp64 longti_r;
        fp32 alti_m;
    }way_point_t;

    way_point_t way_point_data[5];

    sdk_std_msg_t UavInfo;
    DJI_Pro_Get_Broadcast_Data(&UavInfo);


    DJI_CalPos(UavInfo.pos.lati,UavInfo.pos.longti,UavInfo.pos.alti,\
               0, 1.902113*5, 0,\
               &way_point_data[0].lati_r,&way_point_data[0].longti_r,&way_point_data[0].alti_m);

    DJI_CalPos(way_point_data[0].lati_r,way_point_data[0].longti_r,way_point_data[0].alti_m,\
               -1.1180339*5, -1.538842*5, 0,\
               &way_point_data[1].lati_r,&way_point_data[1].longti_r,&way_point_data[1].alti_m);

    DJI_CalPos(way_point_data[1].lati_r,way_point_data[1].longti_r,way_point_data[1].alti_m,\
               1.8090169*5, 0.587785*5, 0,\
               &way_point_data[2].lati_r,&way_point_data[2].longti_r,&way_point_data[2].alti_m);

    DJI_CalPos(way_point_data[2].lati_r,way_point_data[2].longti_r,way_point_data[2].alti_m,\
               -1.8090169*5, 0.587785*5, 0,\
               &way_point_data[3].lati_r,&way_point_data[3].longti_r,&way_point_data[3].alti_m);

    DJI_CalPos(way_point_data[3].lati_r,way_point_data[3].longti_r,way_point_data[3].alti_m,\
               1.1180339*5, -1.538842*5, 0,\
               &way_point_data[4].lati_r,&way_point_data[4].longti_r,&way_point_data[4].alti_m);

    for(int i = 0; i < 5; i++)
    {
        DJI_GotoPos(way_point_data[i].lati_r, way_point_data[i].longti_r,\
                    way_point_data[i].alti_m,0.3);
    }
}

static void * DJI_Sample_Funny_Ctrl_Thread_Func(void *arg)
{
    char *param = (char *)arg;
    int i;
    attitude_data_t user_ctrl_data;
    /* takeoff */
    DJI_Pro_Status_Ctrl(4,0);
    sleep(8);
    /* attitude control, go up */
    for(i = 0; i < 100; i ++)
    {
        user_ctrl_data.ctrl_flag = 0x40;
        user_ctrl_data.roll_or_x = 0;
        user_ctrl_data.pitch_or_y = 0;
        if(i < 90)
            user_ctrl_data.thr_z = 2.0;
        else
            user_ctrl_data.thr_z = 0.0;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
        usleep(20000);
    }
    sleep(2);
    /* do funny behavior*/
    switch(*param)
    {
    case DRAW_CIRCLE_SAMPLE:
        DJI_Sample_Circle_By_Pos();
        break;
    case DRAW_SQUARE_SAMPLE:
        DJI_Sample_Square_By_Pos();
        break;
    case WAY_POINT_SAMPLE:
        DJI_Sample_Way_Point();
        break;
    }
    /* landing */
    sleep(2);
    DJI_Pro_Status_Ctrl(6,0);
    atti_ctrl_sample_flag = -1;
}

int DJI_Sample_Funny_Ctrl(char cmd)
{
    static char s_type = 0;
    if(atti_ctrl_sample_flag == 0)
    {
        return -1;
    }
    atti_ctrl_sample_flag = 0;

    s_type = cmd;

    if(DJI_Sample_Create_Thread(DJI_Sample_Funny_Ctrl_Thread_Func,&s_type) != 0)
    {
        return -1;
    }
    return 0;
}

#if (defined(PLATFORM_LINUX) && defined(TINYXML_CFG))
int DJI_Pro_Get_Cfg(int *baud, char *dev,unsigned int *app_id, unsigned int *app_api_level,char *app_key)
{
	XMLDocument xml_file;
	const XMLAttribute *xml_attr;
	xml_file.LoadFile("config.xml");
	if(xml_file.ErrorID())
	{
		printf("%s:%d:Load user config.xml file ERROR,using default user setting\n",__func__,__LINE__);
		return -1;
	}
	xml_attr = xml_file.RootElement()->FirstChildElement("UART")->FirstAttribute();
    if(baud)
        *baud = atoi(xml_attr->Value());
    if(dev)
        strcpy(dev,xml_attr->Next()->Value());

	xml_attr = xml_file.RootElement()->FirstChildElement("Account")->FirstAttribute();
    if(app_id)
        *app_id = atoi(xml_attr->Value());
    if(app_api_level)
        *app_api_level = atoi(xml_attr->Next()->Value());
    if(app_key)
        strcpy(app_key,xml_attr->Next()->Next()->Value());
	return 0;
}

void DJI_Sample_Drone_Status_Query(void)
{
    unsigned char bat = 0;
    api_ctrl_info_data_t ctrl_info;
    char info[][32]={{"Remote controller"},
                     {"Mobile device"},
                     {"Onboard device"},
                    };
    printf("--  Current status info: --\n");

    DJI_Pro_Get_Bat_Capacity(&bat);
    printf("Battery capacity:[%d%%]\n",bat);
    DJI_Pro_Get_CtrlInfo(&ctrl_info);
    printf("Control device:[%s]\n",*(info + ctrl_info.cur_ctrl_dev_in_navi_mode));
}
#endif
int DJI_Sample_Setup(void)
{
#if (defined(PLATFORM_LINUX) && defined(TINYXML_CFG))
	int ret;
    int baudrate = 115200;
    char uart_name[32] = {"/dev/ttyUSB0"};

    if(DJI_Pro_Get_Cfg(&baudrate,uart_name,NULL,NULL,NULL) == 0)
	{
		/* user setting */
		printf("\n--------------------------\n");
		printf("uart_baud=%d\n",baudrate);
		printf("uart_name=%s\n",uart_name);
		printf("--------------------------\n");
	}
    ret = Pro_Hw_Setup(uart_name,baudrate);
	if(ret < 0)
		return ret;
#endif
    DJI_Pro_Setup(NULL);
	return 0;
}
