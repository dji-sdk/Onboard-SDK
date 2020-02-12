/*! @file dji_stm32_helpers.cpp
 *  @version 3.3
 *  @date Sep 12 2017
 *
 *  @brief
 *  Helper functions to handle serial port validation and vehicle initialization
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_stm32_helpers.hpp"
#include "osdkhal_stm32.h"
#include "osdkosal_stm32.h"

STM32Setup::UartChannelInitParams STM32Setup::uartChnParams[] = {
  {UART_PORT, 921600, FC_UART_CHANNEL_ID},
  {ACM_PORT,  921600, USB_ACM_CHANNEL_ID},
};

STM32Setup::STM32Setup() {
  this->vehicle = NULL;
  setupEnvironment();
}

STM32Setup::~STM32Setup()
{
  if (vehicle){
    delete (vehicle);
    vehicle = NULL;
  }
}

static E_OsdkStat OsdkUser_Console(const uint8_t *data, uint16_t dataLen) {
  //printf("%s", data);
  DJI::OSDK::Log::instance().print("%s", data);
  return OSDK_STAT_OK;
}

void STM32Setup::initVehicle() {
  /*! Linker initialization */
  Linker *linker = new (std::nothrow) Linker();
  if (!linker) {
    DERROR("Failed to allocate memory for Linker!");
    this->vehicle = NULL;
    return;
  }

  if (!linker->init()) {
    DERROR("Failed to initialize Linker!");
    delete linker;
    this->vehicle = NULL;
    return;
  }

  for (int i = 0; i < (sizeof(uartChnParams) / sizeof(UartChannelInitParams)); i++) {
    if (!linker->addUartChannel(uartChnParams[i].device,
                                uartChnParams[i].baudrate,
                                uartChnParams[i].id)) {
      DERROR("Failed to initialize Linker uart channel :");
      DERROR("device : %s, baudrate : %d, channel id : 0x%08X", uartChnParams[i].device,
             uartChnParams[i].baudrate, uartChnParams[i].id);
    } else {
      DSTATUS("Success to initialize Linker uart channel :");
      DSTATUS("device : %s, baudrate : %d, channel id : 0x%08X", uartChnParams[i].device,
              uartChnParams[i].baudrate, uartChnParams[i].id);
    }
  }

  /*! Vehicle initialization */
  this->vehicle = new Vehicle(linker);
  if (!this->vehicle) {
    DERROR("Failed to allocate memory for Vehicle!");
    return;
  }
  
  if (!this->vehicle->initLegacyLinker())
  {
    DERROR("Failed to initialize legacyLinker!\n");
    return;
  }
}

void
STM32Setup::setupEnvironment()
{
  static T_OsdkLoggerConsole printConsole;
  printConsole.consoleLevel = OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO;
  printConsole.func = OsdkUser_Console;

  static T_OsdkHalUartHandler halUartHandler;
  halUartHandler.UartInit = OsdkSTM32_UartInit;
  halUartHandler.UartWriteData = OsdkSTM32_UartSendData;
  halUartHandler.UartReadData = OsdkSTM32_UartReadData;

  static T_OsdkOsalHandler osalHandler;
  osalHandler.TaskCreate = OsdkSTM32_TaskCreate;
  osalHandler.TaskDestroy = OsdkSTM32_TaskDestroy;
  osalHandler.TaskSleepMs = OsdkSTM32_TaskSleepMs;
  osalHandler.MutexCreate = OsdkSTM32_MutexCreate;
  osalHandler.MutexDestroy = OsdkSTM32_MutexDestroy;
  osalHandler.MutexLock = OsdkSTM32_MutexLock;
  osalHandler.MutexUnlock = OsdkSTM32_MutexUnlock;
  osalHandler.SemaphoreCreate = OsdkSTM32_SemaphoreCreate;
  osalHandler.SemaphoreDestroy = OsdkSTM32_SemaphoreDestroy;
  osalHandler.SemaphoreWait = OsdkSTM32_SemaphoreWait;
  osalHandler.SemaphoreTimedWait = OsdkSTM32_SemaphoreTimedWait;
  osalHandler.SemaphorePost = OsdkSTM32_SemaphorePost;
  osalHandler.GetTimeMs = OsdkSTM32_GetTimeMs;
#ifdef OS_DEBUG
  osalHandler.GetTimeUs = OsdkSTM32_GetTimeUs;
#endif
  osalHandler.Malloc = OsdkSTM32_Malloc;
  osalHandler.Free = OsdkSTM32_Free;

  if(DJI_REG_LOGGER_CONSOLE(&printConsole) != true) {
    DERROR("logger console register fail");
  };

  if(DJI_REG_UART_HANDLER(&halUartHandler) != true) {
    DERROR("Uart handler register fail");
  };

  if(DJI_REG_OSAL_HANDLER(&osalHandler) != true) {
    DERROR("Osal handler register fail");
  };
}

