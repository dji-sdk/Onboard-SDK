/*
 * DJI_Pro_Test.h
 *
 *  Created on: 7 Apr, 2015
 *      Author: wuyuwei
 */

#ifndef DJI_PRO_TEST_H_
#define DJI_PRO_TEST_H_
#include "DJI_Pro_App.h"

#define DEFAULT_KEY     "DJI-DEMO AES256 KEY-lala-haha-MA"

/* external functions */
void DJI_cmd_data_callback(unsigned char data);
void DJI_ctrl_mode_callback(int data);
void DJI_simple_task_callback(int data);
void DJI_Onboard_API_Takeoff(void);
void DJI_Onboard_API_Landing(void);
void DJI_Onboard_API_Gohome(void);
void DJI_Onboard_API_Control(unsigned char data);
void DJI_Onboard_API_Activation(void);
void DJI_query_callback(void);
int DJI_Pro_Test_Setup(void);
void DJI_Get_Info(unsigned char *battery, unsigned char *actavation_status, unsigned char *ctrl_device);
void test_all(bool&);

/* internal functions */
unsigned int Get_Time(void);
void spin_callback(void);
extern int simple_task_num;
extern activation_data_t activation_msg;
extern const char *key;

#endif /* DJI_PRO_TEST_H_ */
