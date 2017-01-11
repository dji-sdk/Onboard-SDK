#ifndef DJI_APITEST_H
#define DJI_APITEST_H

#include "DJI_API.h"
#include "DJI_Common.h"
#include "DJI_Flight.h"
#include "DJI_HotPoint.h"
#include "DJI_VirtualRC.h"
#include "DJI_WayPoint.h"
#include "gtest/gtest.h"
#if WIN32
#include <io.h>
#include <windows.h>
#include "DJI_HardDriver_Qt.h"
#else
#include <unistd.h>
#include "DJI_Environment.h"
#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#endif
#include <time.h>
#include <ctime>
#include <fstream>

using namespace DJI::onboardSDK;

extern DJI_Environment* environment;

const char M100_31_FW_VERSION[32] = "SDK-v1.0 BETA M100-03.01.01.00";
const char A3_31_FW_VERSION[32] = "SDK-v1.0 BETA A3-03.01.100.00";
const char A3_32_FW_VERSION[32] = "versionA3_32";
 
class DJI_APITest : public ::testing::Test {
 public:
  DJI_APITest();
  virtual ~DJI_APITest();
  virtual void SetUp();
  virtual void TearDown();

 protected:
  ::testing::AssertionResult read();
  void start_simulator();
  void stop_simulator();
  void setControlStandard();
  void releaseControlStandard();
  void activateDroneStandard();
  unsigned short activateDroneStandard(int l_app_id, std::string l_enc_key,
                                       Version version);
  void std_sleep();

  CoreAPI* api;
  ActivateData user_ack_data;
  Version targetVersion;
  VersionData versionData;
  LinuxThread* readThread;

  // Activation data
  int app_id;
  std::string enc_key;

  // Simulator data
  std::string sim_args;
  std::string sim_path;
  unsigned int sim_timeout_s;
  bool sim_control_enabled;

  /*
   * Test data
   */

  static const int wait_timeout = 10;
  unsigned short ack;
  HardDriver* serialDevice;
  std::string sDevice;
  unsigned int baudrate;

};

#endif

