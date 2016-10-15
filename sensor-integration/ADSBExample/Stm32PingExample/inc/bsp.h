/*! @file bsp.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Helper functions for board STM32F4Discovery
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef STM32PINGEXAMPLE_SRC_BSP_H_
#define STM32PINGEXAMPLE_SRC_BSP_H_

#include "stm32f4xx.h"
#include "DJI_API.h"

void boardInit();
void delay_nms(uint16_t time);

#endif /* STM32PINGEXAMPLE_SRC_BSP_H_ */
