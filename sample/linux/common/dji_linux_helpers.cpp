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

using namespace DJI::OSDK;

LinuxSetup::LinuxSetup(int argc, char** argv, bool enableAdvancedSensing)
{
  this->functionTimeout     = 1; // second
  this->vehicle             = nullptr;
  this->environment         = nullptr;
  this->testSerialDevice    = nullptr;
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
  if (environment){
    delete (environment);
    environment = nullptr;
  }
  if (testSerialDevice){
    delete (testSerialDevice);
    testSerialDevice = nullptr;
  }
}

void
LinuxSetup::setupEnvironment(int argc, char** argv)
{

  // Config file loading
  const char* acm_dev_prefix = "/dev/ttyACM";
  std::string config_file_path;
  std::string acm_device_path = "";

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
  bool threadSupport = true;
  this->vehicle      = new Vehicle(environment->getDevice().c_str(),
                                   environment->getBaudrate(),
                                   threadSupport,
                                   this->useAdvancedSensing);

  // Check if the communication is working fine
  if (!vehicle->protocolLayer->getDriver()->getDeviceStatus())
  {
    std::cout << "Comms appear to be incorrectly set up. Exiting." << std::endl;
    delete (vehicle);
    delete (environment);
    this->vehicle     = nullptr;
    this->environment = nullptr;
  }

  // Activate
  activateData.ID = environment->getApp_id();
  char app_key[65];
  activateData.encKey = app_key;
  strcpy(activateData.encKey, environment->getEnc_key().c_str());
  activateData.version = vehicle->getFwVersion();
  ACK::ErrorCode ack   = vehicle->activate(&activateData, functionTimeout);

  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    delete (vehicle);
    delete (environment);
    this->environment = nullptr;
    this->vehicle     = nullptr;
  }

}

bool
LinuxSetup::validateSerialPort()
{
  static const int BUFFER_SIZE = 2048;

  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  if (!testSerialDevice->setSerialPureTimedRead())
  {
    DERROR("Failed to set up port for timed read.\n");
    return (false);
  };
  usleep(100000);
  if (testSerialDevice->serialRead(buf, BUFFER_SIZE))
  {
    DERROR("Succeeded to read from serial device\n");
  }
  else
  {
    DERROR("\"Failed to read from serial device. The Onboard SDK is not "
             "communicating with your drone. \n");
    // serialDevice->unsetSerialPureTimedRead();
    return (false);
  }

  // If we reach here, _serialRead succeeded.
  int baudCheckStatus = testSerialDevice->checkBaudRate(buf);
  if (baudCheckStatus == -1)
  {
    DERROR("No data on the line. Is your drone powered on?\n");
    return false;
  }
  if (baudCheckStatus == -2)
  {
    DERROR("Baud rate mismatch found. Make sure DJI Assistant 2 has the same "
             "baud setting as the one in User_Config.h\n");
    return (false);
  }
  // All the tests passed and the serial device is properly set up
  testSerialDevice->unsetSerialPureTimedRead();
  return (true);
}
