#include "Receive.h"
#include "main.h"

using namespace DJI::onboardSDK;

#define MAX_RECEIVE 50		//most receiveable length
unsigned char Rx_buff[MAX_RECEIVE] = {0};		//receive buffer area
unsigned char come_data=0;
unsigned char Rx_length=0;	//length of receive buffer when finish receiving								
int Rx_adr=0;				//receive buffer address
int Rx_Handle_Flag=0;		
extern CoreAPI *coreApi;
extern Flight flight ;
extern FlightData flightData ;
BroadcastData broadcastdata;

float Hex2int(char HighBit,char LowBit)
{	//turn two 8-bits hex number into a signed integer,which range of-32767~32727
	if(HighBit&0x80)
	{//
		return (-(((HighBit&0x7f)<<8)|LowBit)/100);
	}
	else
	{//MSB is 1 means a negative number    e.g MSB means most significant bit
		return ((((HighBit&0x7f)<<8)|LowBit)/100);
	}
}


// Recive Buffer Handler Function
void Rx_buff_Handler()      //when USART recive a 0xfe,start handle command 
{  													//in main function
	
	int i=0;
	if(Rx_buff[0]!=0xFA)
	{
      return;       //fomat erro, return
	}
	else
	{
		if(Rx_buff[1]==0xFB)   						//if data begin with 0xfa 0xfb,mode I
		{		
				if(Rx_buff[2]==0x00)   				//0x00 to get current version
				{
				  coreApi->getVersion();        
				}
				if(Rx_buff[2]==0x01)					
				{	
				  User_Activate();          //0x01 to activate
				}
				if(Rx_buff[2]==0x02)         //0x02 to obtain control
				{
						if(Rx_buff[3]==0x00)
						{
							coreApi->setControl(0);
						}
						if(Rx_buff[3]==0x01)
						{
							coreApi->setControl(1);
						}					
				}
				if(Rx_buff[2]==0x03)        //0x03 to Arm
				{
					if(Rx_buff[3]==0x00)
					{
						flight.setArm(0);
					}
					if(Rx_buff[3]==0x01)
					{
						flight.setArm(1);
					}	
				}
				if(Rx_buff[2]==0x04)  
				{
					if(Rx_buff[3]==0x01)  
					{   
						flightData.ctrl_flag =Rx_buff[4];  
						flightData.roll_or_x = Hex2int(Rx_buff[5],Rx_buff[6]);
						flightData.pitch_or_y = Hex2int(Rx_buff[7],Rx_buff[8]);
						flightData.thr_z = Hex2int(Rx_buff[9],Rx_buff[10]);
						flightData.yaw = Hex2int(Rx_buff[11],Rx_buff[12]);
						flight.setFlight(&flightData);
						TIM_Cmd(TIM2,ENABLE);
						printf("roll_or_x =%f\n",Hex2int(Rx_buff[5],Rx_buff[6]));
						printf("pitch_or_y =%f\n",Hex2int(Rx_buff[7],Rx_buff[8]));
						printf("thr_z =%f\n",Hex2int(Rx_buff[9],Rx_buff[10]));
						printf("yaw =%f\n\n",Hex2int(Rx_buff[11],Rx_buff[12]));
					}
					if(Rx_buff[3]==0x00)
					{
						flightData.ctrl_flag = 0x91;   // binary 10 01 00 01
						flightData.roll_or_x = 10;
						flightData.pitch_or_y = 10;
						flightData.thr_z = 20;
						flightData.yaw = 100;
						flight.setFlight(&flightData);
					}
						if(Rx_buff[3]==0x02)
						{														//for display ur hex2int
							printf("roll_or_x =%f\n",Hex2int(Rx_buff[5],Rx_buff[6]));
							printf("pitch_or_y =%f\n",Hex2int(Rx_buff[7],Rx_buff[8]));
							printf("thr_z =%f\n",Hex2int(Rx_buff[9],Rx_buff[10]));
							printf("yaw =%f\n\n",Hex2int(Rx_buff[11],Rx_buff[12]));
							TIM_Cmd(TIM2,ENABLE);
						}
				}
			if(Rx_buff[2]==0x05)  
			{	
				if(Rx_buff[3]==0x01)  
				{
					flight.task(flight.TASK_GOHOME);
				}
				if(Rx_buff[3]==0x02)  
				{
					flight.task(flight.TASK_TAKEOFF);
				}
				if(Rx_buff[3]==0x03)  
				{
					flight.task(flight.TASK_LANDING);
				}
			}
			if(Rx_buff[2]==0x06)                    //VirtualRC  begin with 0x06
			{
				if(Rx_buff[3]==0x00)  						//0x06 0x00 to stop VRC
				{
					VRCResetData();
					TIM_Cmd(TIM1,DISABLE);
				}
				if(Rx_buff[3]==0x02)  					//0x06 0x01 to turn VRC to F gear
				{				
					VRC_TakeControl();
					TIM_Cmd(TIM1,ENABLE);
				}
				if(Rx_buff[3]==0x01)  				//0x06 0x02 to reset data
				{
					VRCResetData();			
					TIM_Cmd(TIM1,ENABLE);
				}
			}
			if(Rx_buff[2]==0x07)             //HotPoint test    07
			{
				tryHotpoint();
			}
			if(Rx_buff[2]==0x08)             //test BroadCastData
			{
				broadcastdata = coreApi->getBroadcastData();
				printf("TimeStamp is %d\r\n",broadcastdata.timeStamp);
				printf("Battery capacity remains %d percent\r\n",broadcastdata.capacity);
			}
		}
		else if(Rx_buff[1]==0xFC)   
		{														//if data begin with 0xff 0xfc,mode II
		}
		else
		{			  
			return;  		//there is only two mode now,if it's none of them,it would be 
									//an error,return.
		}
	}
	
//	//empty receive buffer
//	for(i=0;i<MAX_RECEIVE;i++)   
//	{
//		Rx_buff[i]=0;
//	}
}
