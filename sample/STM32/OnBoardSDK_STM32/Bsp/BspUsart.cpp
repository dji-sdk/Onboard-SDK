#include "BspUsart.h"
#include "DJI_API.h"
#include "DJI_HardDriver.h"
#include "DJI_Flight.h"
extern int Rx_Handle_Flag;		

using namespace DJI::onboardSDK;
extern CoreAPI defaultAPI ;
extern CoreAPI *coreApi ;
extern Flight flight ;
extern FlightData flightData ;

extern unsigned char come_data;
extern unsigned char Rx_length;	 
extern int Rx_adr;				 
extern int Rx_Handle_Flag;		
extern unsigned char Rx_buff[];		 

void USART2_Gpio_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA  , ENABLE);
	  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);		//tx
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);		//rx
	
}

void USART3_Gpio_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB  , ENABLE);
	  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);		//tx
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);		//rx
	
}


void USART2_Config(void)
{ 
    USART2_Gpio_Config();
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		USART_InitTypeDef USART_InitStructure;
		
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART2,&USART_InitStructure);   
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
    
    USART_Cmd(USART2,ENABLE);
   
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)!=SET); 
	 
}

void USART3_Config(void)
{
		USART3_Gpio_Config(); 
	
		USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
    USART_InitStructure.USART_BaudRate = 230400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART3,&USART_InitStructure);   
    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
    
    USART_Cmd(USART3,ENABLE);
    while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)!=SET); 

}
void USARTxNVIC_Config()
{
    NVIC_InitTypeDef NVIC_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure2;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannel  = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    NVIC_Init(&NVIC_InitStructure);
	
	
    NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure2.NVIC_IRQChannel  = USART2_IRQn;
    NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
        
    NVIC_Init(&NVIC_InitStructure2);
}

void UsartConfig()
{
  USART2_Config();
  USART3_Config();
  USARTxNVIC_Config();
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void USART3_IRQHandler(void)
{
		if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE)==SET)
	{
		coreApi->byteHandler(USART_ReceiveData(USART3));             //Data from M100 were committed to "byteHandler"
	}
}

//command from PC would be handle here
void USART2_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)==SET)
	{
		 come_data = USART_ReceiveData(USART2);   //come_data is a ONE-BYTE temp-container
		
		if(Rx_adr == 0)		 
		{
			if(come_data == 0xFA)
			{
				Rx_buff[0] = 0xFA;
				Rx_adr =1;			
			}
			else
			{}       
		}
		else
		{
			if(come_data == 0xfe)	  //receive a 0xfe would lead to a command-execution
			{
				Rx_buff[Rx_adr] = come_data;
				Rx_length = Rx_adr+1;
				Rx_adr = 0;
				Rx_Handle_Flag = 1;
			}
			else		 
			{			
				Rx_buff[Rx_adr] = come_data;
				Rx_adr++;
			}
		}
	}
}

#ifdef __cplusplus
}
#endif //__cplusplus
