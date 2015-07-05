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
void DJI_Onboard_API_UAV_Control(unsigned char data);
void DJI_Onboard_API_Activation(void);
void DJI_Onboard_API_Status_Query(void);
int DJI_Pro_Test_Setup(void);
/* internal functions */
unsigned int Get_Time(void);
void spin_callback(void);

#endif /* DJI_PRO_TEST_H_ */
