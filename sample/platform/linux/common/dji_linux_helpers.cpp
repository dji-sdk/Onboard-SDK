/*! @file dji_linux_helpers.cpp
 *  @version 4.0.0
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

#include "dji_linux_helpers.hpp"
#include "osdkhal_linux.h"
#include "osdkosal_linux.h"

static E_OsdkStat OsdkUser_Console(const uint8_t *data, uint16_t dataLen)
{
  return OSDK_STAT_OK;
  printf("%s", data);

  return OSDK_STAT_OK;
}

using namespace DJI::OSDK;

LinuxSetup::LinuxSetup(int argc, char **argv, bool enableAdvancedSensing)
    : Setup(enableAdvancedSensing)
{
  functionTimeout = 1; //second
  setupEnvironment(argc, argv);
  initVehicle();
}

LinuxSetup::~LinuxSetup()
{
  if (vehicle){
    delete (vehicle);
    vehicle = nullptr;
  }
  if (environment){
    delete (environment);
    environment = nullptr;
  }
}

void
LinuxSetup::setupEnvironment(int argc, char** argv)
{
  static T_OsdkLoggerConsole printConsole = {
      .consoleLevel = OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO,
      .func = OsdkUser_Console,
  };

  static T_OsdkHalUartHandler halUartHandler = {
      .UartInit = OsdkLinux_UartInit,
      .UartWriteData = OsdkLinux_UartSendData,
      .UartReadData = OsdkLinux_UartReadData,
      .UartClose = OsdkLinux_UartClose,
  };

#ifdef ADVANCED_SENSING
  static T_OsdkHalUSBBulkHandler halUSBBulkHandler = {
      .USBBulkInit = OsdkLinux_USBBulkInit,
      .USBBulkWriteData = OsdkLinux_USBBulkSendData,
      .USBBulkReadData = OsdkLinux_USBBulkReadData,
      .USBBulkClose = OsdkLinux_USBBulkClose,
  };
#endif

  static T_OsdkOsalHandler osalHandler = {
      .TaskCreate = OsdkLinux_TaskCreate,
      .TaskDestroy = OsdkLinux_TaskDestroy,
      .TaskSleepMs = OsdkLinux_TaskSleepMs,
      .MutexCreate = OsdkLinux_MutexCreate,
      .MutexDestroy = OsdkLinux_MutexDestroy,
      .MutexLock = OsdkLinux_MutexLock,
      .MutexUnlock = OsdkLinux_MutexUnlock,
      .SemaphoreCreate = OsdkLinux_SemaphoreCreate,
      .SemaphoreDestroy = OsdkLinux_SemaphoreDestroy,
      .SemaphoreWait = OsdkLinux_SemaphoreWait,
      .SemaphoreTimedWait = OsdkLinux_SemaphoreTimedWait,
      .SemaphorePost = OsdkLinux_SemaphorePost,
      .GetTimeMs = OsdkLinux_GetTimeMs,
#ifdef OS_DEBUG
      .GetTimeUs = OsdkLinux_GetTimeUs,
#endif
      .Malloc = OsdkLinux_Malloc,
      .Free = OsdkLinux_Free,
  };

  if(DJI_REG_LOGGER_CONSOLE(&printConsole) != true) {
    throw std::runtime_error("logger console register fail");
  }

  if(DJI_REG_UART_HANDLER(&halUartHandler) != true) {
    throw std::runtime_error("Uart handler register fail");
  }

#ifdef ADVANCED_SENSING
  if(DJI_REG_USB_BULK_HANDLER(&halUSBBulkHandler) != true) {
    throw std::runtime_error("USB Bulk handler register fail");
  };
#endif

  if(DJI_REG_OSAL_HANDLER(&osalHandler) != true) {
    throw std::runtime_error("Osal handler register fail");
  }

  // Config file loading
  const char* acm_dev_prefix = "/dev/ttyACM";
  std::string config_file_path;
  std::string acm_device_path = "";
  std::string sample_case_name = "";

  // Find config file among given parameters
  for(int i = 1; i < argc; i++)
  {
    std::ifstream ifs( argv[i] );
    if (ifs.is_open())
    {
      config_file_path = argv[i];
      if(config_file_path.find(".txt") != std::string::npos)
      {
        // Found config file
        break;
      }
    }
  }

  for(int i = 1; i < argc; i++)
  {
    if(strncmp(argv[i], acm_dev_prefix, strlen(acm_dev_prefix)) == 0)
    {
      std::cout << "Find the target ttyACM device , path:" <<  argv[i] << std::endl;
      acm_device_path = argv[i];
      break;
    }

    if (strlen(argv[i]) == 1)
    {
      std::cout << "Find sample case parameter : " <<  argv[i] << std::endl;
      sample_case_name = argv[i];
      break;
    }
  }

  if (!config_file_path.empty())
  {
    std::ifstream fStream(config_file_path.c_str());
    if (!fStream.good())
      throw std::runtime_error("User configuration file not found");
  }
  else
  {
    config_file_path = DJI_Environment::findFile("UserConfig.txt");

    if (config_file_path.empty())
      throw std::runtime_error("User configuration file not found");
  }

  this->environment = new DJI_Environment(config_file_path);
  if (!environment->getConfigResult())
  {
    // We were unable to read the config file. Exit.
    throw std::runtime_error(
        "User configuration file is not correctly formatted.");
  }

  /* set ttyACM device */
  if(acm_device_path != "")
  {
    std::cout << "Set ACM device:" << acm_device_path.c_str() << std::endl;
    this->environment->setDeviceAcm(acm_device_path);
  }

  if(!sample_case_name.empty())
  {
    std::cout << "Set sample case:" << sample_case_name.c_str() << std::endl;
    this->environment->setSampleCase(sample_case_name);
  }
}

bool
LinuxSetup::initVehicle()
{
  ACK::ErrorCode ack;

  /*! Linker initialization */
  if (!initLinker()) {
    DERROR("Failed to initialize Linker");
    return false;
  }

  /*! Linker add uart channel */
  if (!addFCUartChannel(environment->getDevice().c_str(),
                        environment->getBaudrate())) {
    DERROR("Failed to initialize Linker channel");
    return false;
  }

  /*! Linker add USB acm channel */
  if (!addUSBACMChannel(environment->getDeviceAcm().c_str(),
                        environment->getACMDefaultBaudrate())) {
    DERROR("Failed to initialize ACM Linker channel!");
  }

  /*! Vehicle initialization */
  if (!linker)
  {
    DERROR("Linker get failed.");
    goto err;
  }

  vehicle = new Vehicle(linker);
  if (!vehicle)
  {
    DERROR("Vehicle create failed.");
    goto err;
  }

  // Activate
  activateData.ID = environment->getApp_id();
  char app_key[65];
  activateData.encKey = app_key;
  strcpy(activateData.encKey, environment->getEnc_key().c_str());
  activateData.version = vehicle->getFwVersion();

  ack = vehicle->activate(&activateData, functionTimeout);
  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    goto err;
  }

  if (!vehicle->isM300()) {
    vehicle->setUSBFlightOn(true);
  }

  return true;

  err:
  if (vehicle) delete (vehicle);
  if (environment) delete (environment);
  this->environment = nullptr;
  this->vehicle     = nullptr;
  return false;
}
