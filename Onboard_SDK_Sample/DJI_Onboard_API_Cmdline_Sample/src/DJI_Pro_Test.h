/*
 * DJI_Pro_Test.h
 *
 *  Created on: 7 Apr, 2015
 *      Author: wuyuwei
 */

#ifndef DJI_PRO_TEST_H_
#define DJI_PRO_TEST_H_


/* external functions */
void DJI_Onboard_API_Simple_Task(int data);
void DJI_Onboard_API_Control(unsigned char data);
void DJI_Onboard_API_UAV_Control(unsigned char arg);
void DJI_Onboard_API_Activation(void);
void DJI_Onboard_API_Status_Query(void);
int DJI_Pro_Test_Setup(void);
void DJI_Get_Info(unsigned char *battery, unsigned char *actavation_status, unsigned char *ctrl_device);
int DJI_Onboard_API_Gimbal_Task(void);

/* internal functions */
unsigned long int Get_Time(void);
void spin_callback(void);


#endif /* DJI_PRO_TEST_H_ */
