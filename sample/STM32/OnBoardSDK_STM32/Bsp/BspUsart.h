#ifndef _BSPUSART_H
#define _BSPUSART_H
#include <stm32f4xx.h>
//#include "stdarg.h"
#include "stdio.h"
void USART2_Config(void);
void USART3_Config(void);
void USARTxNVIC_Config(void);
void UsartConfig(void);
void NVIC_Config(void);
void Rx_buff_Handler() ;
#endif  //_USART_H
