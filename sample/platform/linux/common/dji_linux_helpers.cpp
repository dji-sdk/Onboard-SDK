/*! @file dji_linux_helpers.cpp
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

#include <dji_linux_helpers.hpp>
#include "osdkhal_linux.h"
#include "osdkosal_linux.h"

static E_OsdkStat OsdkUser_Console(const uint8_t *data, uint16_t dataLen) 
{
  printf("%s", data);

  return OSDK_STAT_OK;
}

static T_OsdkLoggerConsole printConsole = {
    .consoleLevel = OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO,
    .func = OsdkUser_Console,
};

static T_OsdkHalUartHandler halUartHandler = {
    .UartInit = OsdkLinux_UartInit,
    .UartWriteData = OsdkLinux_UartSendData,
    .UartReadData = OsdkLinux_UartReadData,
};

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

using namespace DJI::OSDK;

LinuxSetup::LinuxSetup(int argc, char** argv, bool enableAdvancedSensing)
{
  this->functionTimeout     = 1; // second
  this->vehicle             = nullptr;
  this->platform            = nullptr;
  this->environment         = nullptr;
  this->useAdvancedSensing  = enableAdvancedSensing;

  setupEnvironment(argc, argv);
  initVehicle();
}

LinuxSetup::~LinuxSetup()
{
  if (vehicle){
    delete (vehicle);
    vehicle = nullptr;
  }
  if (platform){
    delete (platform);
    platform = nullptr;
  }
  if (environment){
    delete (environment);
    environment = nullptr;
  }
}

void
LinuxSetup::setupEnvironment(int argc, char** argv)
{

  this->platform = new Platform();

  if(platform->registerLoggerConsole(&printConsole) != true) {
    throw std::runtime_error("logger console register fail");
  };

  if(platform->registerHalUartHandler(&halUartHandler) != true) {
    throw std::runtime_error("Uart handler register fail");
  };

  if(platform->registerOsalHandler(&osalHandler) != true) {
    throw std::runtime_error("Osal handler register fail");
  };

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
    printf("argv[%d] = %s\n", i, argv[i]);
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

  /*  this->testSerialDevice = new LinuxSerialDevice(
      environment->getDevice().c_str(), environment->getBaudrate());
    testSerialDevice->init();
    bool setupStatus = validateSerialPort();

    if (!setupStatus)
    {
      delete (testSerialDevice);
      delete (environment);
      return NULL;
    }
    else
    {
      delete (testSerialDevice);
    }
  */
}

void
LinuxSetup::initVehicle()
{

  this->vehicle      = new Vehicle(environment->getDevice().c_str(),
                                   environment->getBaudrate(),
                                   getPlatform());

  if(!vehicle->init())
  {
    std::cout << "vehicle init fail. Exiting." << std::endl;
    goto err;
  }
  
  if(!vehicle->linker->addUartChannel(vehicle->device, vehicle->baudRate))
  {
    std::cout << "Uart init fail. Exiting." << std::endl;
    goto err;
  }

  // Activate
  activateData.ID = environment->getApp_id();
  char app_key[65];
  activateData.encKey = app_key;
  strcpy(activateData.encKey, environment->getEnc_key().c_str());
  activateData.version = vehicle->getFwVersion();
  if(!vehicle->activate(&activateData))
  {
    std::cout << "activate fail. Exiting." << std::endl;
    goto err;
  }
  vehicle->activateSemWait();
  if(!vehicle->getActivationStatus())
  {
    std::cout << "activate fail. Exiting." << std::endl;
    goto err;
  }
  return;

err:
  delete (platform);
  delete (vehicle);
  delete (environment);
  this->platform    = nullptr;
  this->environment = nullptr;
  this->vehicle     = nullptr;
}
