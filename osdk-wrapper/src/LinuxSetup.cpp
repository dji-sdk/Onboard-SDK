/*! @file LinuxSetup.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Executes some setup commands that are required by the Onboard SDK
 *  but are not pertinent to the user's work. Also sets up threads and memory.    
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#include "LinuxSetup.h"

using namespace std;

#ifdef LIDAR_LOGGING
boost::mutex io_mutex;
unsigned int controlFlag=0;
boost::condition_variable condition;
unsigned int loggingTypeFlag=2; // 0: pcap only, 1: las file only; 2: both pcap and las files
bool ctrlCOrCloseTerminalFlag = false;
boost::thread workerThread1;
boost::thread workerThread2;
#endif

int setup(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read)
{
  //! Configuration parsing
  int configStatus = parseUserConfig(); 

  if (configStatus == -1)
  {
    return configStatus;
  }

  //! Set device and baudrate as per userConfig
  serialDevice->setDevice(UserConfig::deviceName);
  serialDevice->setBaudrate(UserConfig::baudRate);
  //! Initialize serial driver
  serialDevice->init();

  //! See if the serial port is open
  bool serialStatus = serialDevice->getDeviceStatus();
  if(serialStatus == 0)
  {
    return(-1);
  }

  //! Validate the serial connection
  #ifdef __x86_64__
    serialStatus = validateSerialDevice(serialDevice, api); 
    if(serialStatus == 0)
    {
      return(-1);
    } 
  #endif

  //! Start read thread
  read->createThread();

  VersionData vData = api->getDroneVersion(1);

  //! Attempt Activation
  ackReturnData activationStatus = activate(api);
  if (activationStatus.status == -1)
  {
    return activationStatus.status;
  }

  //! We are successfully activated. Try to take control.
  ackReturnData controlStatus = takeControl(api);
  if (controlStatus.status == -1)
  {
    return controlStatus.status;
  }
  api->getDroneVersion();
  //! Setup completed sucessfully.
  return 1;
}

bool validateSerialDevice(LinuxSerialDevice* serialDevice, CoreAPI* api)
{
  /*! Check if the serial connection is valid*/

  //! Set broadcast frequency to be just timestamp data at 100Hz for a channel check

  uint8_t freq[16];

  freq[0] = BROADCAST_FREQ_100HZ;
  freq[1] = BROADCAST_FREQ_0HZ;
  freq[2] = BROADCAST_FREQ_0HZ;
  freq[3] = BROADCAST_FREQ_0HZ;
  freq[4] = BROADCAST_FREQ_0HZ;
  freq[5] = BROADCAST_FREQ_0HZ;
  freq[6] = BROADCAST_FREQ_0HZ;
  freq[7] = BROADCAST_FREQ_0HZ;
  freq[8] = BROADCAST_FREQ_0HZ;
  freq[9] = BROADCAST_FREQ_0HZ;
  freq[10] = BROADCAST_FREQ_0HZ;
  freq[11] = BROADCAST_FREQ_0HZ; 

  api->setBroadcastFreq(freq);

  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  serialDevice->setSerialPureTimedRead();
  if(serialDevice->serialRead(buf, BUFFER_SIZE))
  {
    API_LOG(serialDevice, STATUS_LOG, "Succeeded to read from serial device\n");
    //! Cleanup - set broadcast frequencies to default once again.
    //api->setBroadcastFreqDefaults();
    //api->setBroadcastFreqToZero();
  }
  else
  {
    API_LOG(serialDevice, ERROR_LOG, "Failed to read from serial device. The Onboard SDK is not communicating with your drone.\n");
    //! Cleanup - set broadcast frequencies to default once again.
    api->setBroadcastFreqDefaults();
    serialDevice->unsetSerialPureTimedRead();
    return (false);
  }

  //If we reach here, _serialRead succeeded. 
  int baudCheckStatus = serialDevice->checkBaudRate(buf);
  if (baudCheckStatus == -1)
  {
    API_LOG(serialDevice, ERROR_LOG, "No data on the line. Is your drone powered on?\n");
    return false;
  }
  if (baudCheckStatus == -2)
  {
    API_LOG(serialDevice, ERROR_LOG, "Baud rate mismatch found. Make sure DJI Assistant 2 has the same baud setting as the one in User_Config.h\n")
    return (false);
  }

  // All the tests passed and the serial device is properly set up 
  serialDevice->unsetSerialPureTimedRead();
  return (true);
}


int parseUserConfig()
{
  readUserConfig();

  std::string droneVerStr, SdkVerStr; 
  switch (UserConfig::targetVersion){
    case DJI::onboardSDK::versionA3_31:
      droneVerStr = "A3";
      SdkVerStr = "3.1";
      break;
    case DJI::onboardSDK::versionA3_32:
      droneVerStr = "A3";
      SdkVerStr = "3.2";
      break;
    case DJI::onboardSDK::versionM100_31:
      droneVerStr = "M100";
      SdkVerStr = "3.1";
      break;
    case DJI::onboardSDK::versionM100_23:
      droneVerStr = "M100";
      SdkVerStr = "2.3";
      break;
    default:
      std::cout << "Invalid Drone version. Please check your UserConfig file.\n";
      std::cout << "Invalid SDK version. Please check your UserConfig file.\n";
      return -1;
      break;
  }

  //Make blocking call to getDroneVersion to see if this setting is correct.
  //We will figure out in the serial config if the baud/port is correct. 
  std::cout << "These are your User_Config settings.\n"
      "Serial port = " << UserConfig::deviceName << "\n"
      "Baudrate = "<< UserConfig::baudRate << "\n"
      "Drone/FC Version: " << droneVerStr << "\n"
      "SDK Version: " << SdkVerStr << "\n"
      "App ID: " << UserConfig::userAppID << "\n"
      "App Key: " << UserConfig::userKey << "\n"
      "\nDoes everything look correct? If not, navigate to Linux/UserConfig.txt and make changes.\n\n";

  usleep(500000);
  return 1;
}

ackReturnData activate(CoreAPI* api)
{
  //! @note Sequence for automatic activation.
  std::cout << "\nAttempting activation..\n";
  
  //! Get data from User_Config.h
  ActivateData activationData;
  char userKeyCstr[65];
  strcpy(userKeyCstr,UserConfig::userKey.c_str());
  activationData.ID = UserConfig::userAppID;
  activationData.encKey = userKeyCstr; 

  //! Tell the SDK our hardware and software versions
  api->setVersion(UserConfig::targetVersion);


  //! Set all broadcast data to zero since we are currently interested only in the activation ACK
  //api->setBroadcastFreqToZero();
  //usleep(5000);

  //! Init variable for return value of blocking activation
  ackReturnData activateAck;
  activateAck.ack = 99; //Default value

  //! Call the blocking activate function. We will not supply our own callback because we can do processing in the main thread with the return value.
  activateAck.ack = api->activate(&activationData, 1);
  //! Give the callback time to return

  switch (activateAck.ack)
  {
    case ACK_ACTIVE_SUCCESS:
      std::cout << "Automatic activation successful." << std::endl;
      usleep(100000);
      activateAck.status = 1;
      return activateAck;
      break;
    case ACK_ACTIVE_NEW_DEVICE:
      std::cout << "Your activation did not go through. \nThis is a new device. \nMake sure DJI GO is turned on and connected to the internet \nso you can contact the server for activation." << std::endl;
      usleep(1000000);
      break;
    case ACK_ACTIVE_PARAMETER_ERROR:
      std::cout << "Your activation did not go through. \nThere was a parameter error. \nPlease check your setup and retry." << std::endl;
      usleep(1000000);
      break;
    case ACK_ACTIVE_ENCODE_ERROR:
      std::cout << "Your activation did not go through. \nThere was an encoding error. \nPlease check your setup and retry." << std::endl;
      usleep(1000000);
      break;
    case ACK_ACTIVE_APP_NOT_CONNECTED:
      std::cout << "Your activation did not go through. \nDJI GO does not seem to be connected. \n Try killing and restarting DJI GO." << std::endl;
      usleep(1000000);
      break;
    case ACK_ACTIVE_NO_INTERNET:
      std::cout << "Your activation did not go through. \nYour mobile device doesn't seem to be connected to the internet. \nPlease check your connection and try again." << std::endl;
      usleep(1000000);
      break;
    case ACK_ACTIVE_SERVER_REFUSED:
      std::cout << "Your activation did not go through. \nThe server refused your credentials. \nPlease check your DJI developer account details in User_Config.h. \n Rebuild (do a `make clean` first) and try again." << std::endl;
      usleep(1000000);
      break;
    case ACK_ACTIVE_ACCESS_LEVEL_ERROR:
      std::cout << "Your activation did not go through. \nYou don't seem to have the right DJI SDK permissions. \nPlease check your DJI developer account details." << std::endl;
      usleep(1000000);
      break;
    case ACK_ACTIVE_VERSION_ERROR:
      std::cout << "Your activation did not go through. \nYour SDK version in User_Config.h does not match the one reported by the drone. \nPlease correct that, rebuild (do a `make clean` first) and try again." << std::endl;
      usleep(1000000);
      break;
    default:
      std::cout << "There was an error with the activation command. This can happen due to a variety of reasons. \n (1)Make sure the baud rate settings in DJI Assistant 2 match those in User_config.h. \n (2) Make sure API control is enabled in DJI Assitant 2. \n If the error persists, raise an issue on Github specifying your onboard platform and a snapshot of DJI Assistant 2 as well as User_Config.h." << std::endl;
      usleep(1000000);
      break;
  }
  activateAck.status = -1;
  return activateAck;

}

ackReturnData takeControl(CoreAPI* api)
{
  unsigned short controlAck;
  //! Obtain code 0x1
  unsigned char data = 0x1;
  ackReturnData takeControlData;
 
  takeControlData.ack = api->setControl(true, 1);
  switch (takeControlData.ack)
  {
    case  ACK_SETCONTROL_NEED_MODE_F:
      std::cout << "Failed to obtain control.\nYour RC mode switch is not in mode F. (Is the RC connected and paired?)" << std::endl;
      takeControlData.status = -1;
      return takeControlData;
    case  ACK_SETCONTROL_NEED_MODE_P:
      std::cout << "Failed to obtain control.\nFor A3 v3.2, your RC needs to be in P mode. (Is the RC connected and paired?)" << std::endl;
      takeControlData.status = -1;
      return takeControlData;
    case ACK_SETCONTROL_OBTAIN_SUCCESS:
      std::cout << "Obtained control successfully."<< std::endl;
      break;
    case ACK_SETCONTROL_OBTAIN_RUNNING:
      std::cout << "Obtain control running.."<< std::endl;
      takeControl(api);
      break;
    case ACK_SETCONTROL_IOC:
      std::cout << "The aircraft is in IOC mode. Cannot obtain control.\nGo to DJI GO and stop all intelligent flight modes before trying this." << std::endl;
      takeControlData.status = -1;
      return takeControlData;
      break;
    default:
      {
        std::cout << "Error in setControl API function." << std::endl;
      }
      break;
  }
  takeControlData.status = 1;
  return takeControlData;
}

//! Non-Blocking

int setupNonBlocking(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read, LinuxThread* callback)
{
  //! Configuration parsing
  int configStatus = parseUserConfig();

  if (configStatus == -1)
  {
    return configStatus;
  }

  //! Set device and baudrate as per userConfig
  serialDevice->setDevice(UserConfig::deviceName);
  serialDevice->setBaudrate(UserConfig::baudRate);
  //! Initialize serial driver
  serialDevice->init();

  //! See if the serial port is open
  bool serialStatus = serialDevice->getDeviceStatus();
  if(serialStatus == 0)
  {
    return(-1);
  }

  //! Validate the serial connection
#ifdef __x86_64__
  serialStatus = validateSerialDevice(serialDevice, api);
  if(serialStatus == 0)
  {
    return(-1);
  }
#endif

  //! Start read thread and callback thread.
  read->createThread();
  callback->createThread();

  activateNonBlocking(api);
  //! We are successfully activated. Try to take control.
  takeControlNonBlocking(api);
  //! Setup completed sucessfully.
  return 1;
}

void activateCallback(CoreAPI* api, Header *protHeader, DJI::UserData data) {

  uint16_t simpAck = api->missionACKUnion.simpleACK;
  switch (simpAck)
  {
    case ACK_ACTIVE_SUCCESS:
      std::cout << "Automatic activation successful." << std::endl;
          usleep(100000);
          break;
    case ACK_ACTIVE_NEW_DEVICE:
      std::cout << "Your activation did not go through. \nThis is a new device. \nMake sure DJI GO is turned on and connected to the internet \nso you can contact the server for activation." << std::endl;
          usleep(1000000);
          break;
    case ACK_ACTIVE_PARAMETER_ERROR:
      std::cout << "Your activation did not go through. \nThere was a parameter error. \nPlease check your setup and retry." << std::endl;
          usleep(1000000);
          break;
    case ACK_ACTIVE_ENCODE_ERROR:
      std::cout << "Your activation did not go through. \nThere was an encoding error. \nPlease check your setup and retry." << std::endl;
          usleep(1000000);
          break;
    case ACK_ACTIVE_APP_NOT_CONNECTED:
      std::cout << "Your activation did not go through. \nDJI GO does not seem to be connected. \n Try killing and restarting DJI GO." << std::endl;
          usleep(1000000);
          break;
    case ACK_ACTIVE_NO_INTERNET:
      std::cout << "Your activation did not go through. \nYour mobile device doesn't seem to be connected to the internet. \nPlease check your connection and try again." << std::endl;
          usleep(1000000);
          break;
    case ACK_ACTIVE_SERVER_REFUSED:
      std::cout << "Your activation did not go through. \nThe server refused your credentials. \nPlease check your DJI developer account details in User_Config.h. \n Rebuild (do a `make clean` first) and try again." << std::endl;
          usleep(1000000);
          break;
    case ACK_ACTIVE_ACCESS_LEVEL_ERROR:
      std::cout << "Your activation did not go through. \nYou don't seem to have the right DJI SDK permissions. \nPlease check your DJI developer account details." << std::endl;
          usleep(1000000);
          break;
    case ACK_ACTIVE_VERSION_ERROR:
      std::cout << "Your activation did not go through. \nYour SDK version in User_Config.h does not match the one reported by the drone. \nPlease correct that, rebuild (do a `make clean` first) and try again." << std::endl;
          usleep(1000000);
          break;
    default:
      std::cout << "There was an error with the activation command. This can happen due to a variety of reasons. \n (1)Make sure the baud rate settings in DJI Assistant 2 match those in User_config.h. \n (2) Make sure API control is enabled in DJI Assitant 2. \n If the error persists, raise an issue on Github specifying your onboard platform and a snapshot of DJI Assistant 2 as well as User_Config.h." << std::endl;
          usleep(1000000);
          break;
  }

}

void activateNonBlocking(CoreAPI* api)
{
  std::cout << "\nAttempting activation..\n";

  //! Get data from User_Config.h
  ActivateData activationData;
  char userKeyCstr[65];
  strcpy(userKeyCstr,UserConfig::userKey.c_str());
  activationData.ID = UserConfig::userAppID;
  activationData.encKey = userKeyCstr;

  //! Tell the SDK our hardware and software versions
  api->setVersion(UserConfig::targetVersion);

  void (*pointerActivationCallback)(CoreAPI*, Header*, DJI::UserData);
  pointerActivationCallback = activateCallback;

  //! Call the non-blocking activate function.
  api->activate(&activationData, pointerActivationCallback, NULL);

}

void takeControlCallback(CoreAPI* api, Header *protHeader, DJI::UserData data) {

  uint16_t simpAck = api->missionACKUnion.simpleACK;
  switch (simpAck)
  {
    case ACK_SETCONTROL_ERROR_MODE:
      if(api->getSDKVersion() != versionA3_32)
        std::cout << "Failed to obtain control.\nYour RC mode switch is not in mode F. (Is the RC connected and paired?)" << std::endl;
      else
        std::cout << "Failed to obtain control.\nYour RC mode switch is not in mode P. (Is the RC connected and paired?)" << std::endl;
      break;
    case ACK_SETCONTROL_OBTAIN_SUCCESS:
      std::cout << "Obtained control successfully."<< std::endl;
          break;
    case ACK_SETCONTROL_OBTAIN_RUNNING:
      std::cout << "Obtain control running.."<< std::endl;
          takeControlNonBlocking(api);
          break;
    case ACK_SETCONTROL_IOC:
      std::cout << "The aircraft is in IOC mode. Cannot obtain control.\nGo to DJI GO and stop all intelligent flight modes before trying this." << std::endl;
          break;
    default:
    {
      std::cout << "Error in setControl API function." << std::endl;
    }
          break;
  }
}
void takeControlNonBlocking(CoreAPI* api)
{
  void (*pointerTakeControlCallback)(CoreAPI*, Header*, DJI::UserData);
  pointerTakeControlCallback = takeControlCallback;
  api->setControl(true, pointerTakeControlCallback, NULL);
}
#ifdef LIDAR_LOGGING
void startLiDARlogging()
{
  boost::lock_guard<boost::mutex> lock(io_mutex);
  boost::thread workerThread1(udpAndLogging);
  boost::thread workerThread2(managementProcessing);
}

void stopLiDARlogging()
{
  controlFlag = 1;
  condition.notify_one();
  workerThread1.interrupt();
  workerThread2.interrupt();
  printf("LiDAR logging exited");
}

void udpAndLogging(void)
{
  std::string logFilePathName="/home/";
  std::string logFilerFolderNamer="/Vlp16_logfiles/";
  int result = mkdir("/home/Vlp16_logfiles/",0700);
  std::string logFileDefaultNamer="Vlp16_log_";
  std::string logFilePcapExtension=".pcap";
  std::string logFileLasExtension=".las";
  std::string logFilePathNamePcap;
  std::string logFilePathNameLas;
  std::string currentUser;

  currentUser = getenv("USER");
  std::cout << "Current User =" << currentUser <<std::endl;

  logFilePathName.append(currentUser);
  logFilePathName.append(logFilerFolderNamer);
  logFilePathName.append(logFileDefaultNamer);

  std::time_t rawTime;
  char tmpBuf[100];

  std::time(&rawTime);
  std::strftime(tmpBuf,100,"%H_%M_%S_%m_%d_%Y",std::localtime(&rawTime));
  std::cout << "Current Local time =";
  std::puts(tmpBuf);

  logFilePathName.append(tmpBuf);
  logFilePathNamePcap=logFilePathName;
  logFilePathNamePcap.append(logFilePcapExtension);
  logFilePathNameLas=logFilePathName;
  logFilePathNameLas.append(logFileLasExtension);

  std::cout << "Current Log file is in =";
  std::cout << logFilePathName <<std::endl;

  unsigned int printingToTerminal=1;   //0: turn it off

  try
  {
    boost::asio::io_service io_service1;
    UDPdriver UDPdriver(io_service1,loggingTypeFlag,logFilePathNamePcap, logFilePathNameLas,printingToTerminal,&controlFlag);
    io_service1.run();

  }
  catch (std::exception& err)
  {
    std::cerr << err.what() << std::endl;
  }

}

void managementProcessing(void)
{
  boost::asio::io_service io_service2;
  boost::asio::signal_set signals(io_service2, SIGINT, SIGTERM, SIGHUP);
  signals.async_wait(signalHandler);
  io_service2.run();
}

void signalHandler(const boost::system::error_code& error, int signal_number)
{
  if (!error)  {
    ctrlCOrCloseTerminalFlag = true;
  }
}
#endif
