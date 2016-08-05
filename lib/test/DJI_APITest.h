#include <gtest/gtest.h>
#include "DJI_API.h"
#include "DJI_WayPoint.h"
#include "DJI_HotPoint.h"
#include "DJI_Flight.h"
#include "DJI_VirtualRC.h"
#if WIN32
#include "DJI_HardDriver_Qt.h"
#include <windows.h>
#include <io.h>
#else 
#include "DJI_HardDriver_Manifold.h"
#include "APIThread.h"
#include <unistd.h>
#endif
#include <ctime>
#include <time.h>
#include <fstream>
#include "json11.hpp"
using namespace DJI::onboardSDK;

#define BUFFER_SIZE 1024
#define DEFAULT_STATUS 11

#define SKIP_TEST() std::cout << "[  SKIPPED ] Feature " <<  "not supported" << std::endl; return;

enum broadcstData: uint16_t
{
  timestamp = 0x0001,
  quaternion= 0x0002,
  acceleration = 0x0004,
  velocity = 0x0008,
  yawRate = 0x0010,
  position = 0x0020,
  magnetometer = 0x0040,
  remoteController = 0x0080,
  gimbal = 0x0100,
  deviceStatus = 0x0200,
  battery = 0x0400,
  controlStatus = 0x0800
};

enum flight_task: int
{
  takeOff = 4,
  goHome = 1,
  landing = 6
};

enum class control_authorization: bool
{
  obtainControl = true,
  releaseControl = false
};

class DJI_APITest : public ::testing::Test
{

protected:
  DJI_APITest ()
  {
    // TODO move device and baudrate initialization
    // to HW specific serial device implementation
    //device = "/dev/ttyTHS1";
    sDevice = "/dev/ttyUSB0";
    baudrate = 230400;

#if WIN32 
    serialDevice = new QHardDriver (new QSerialPort());
#else 
    serialDevice = new HardDriver_Manifold (sDevice, baudrate);
#endif
  }

  virtual ~DJI_APITest()
  {
    readThread->stop();
    delete readThread;
    delete api;
    delete serialDevice;
  }

  // Code here will be called immediately
  // after the constructor but before each test
  virtual void SetUp()
  {
    serialDevice->init();
    ASSERT_EQ(serialDevice->getDevieStatus(), true);

    api = new CoreAPI ((HardDriver*) serialDevice);

    readThread = new APIThread(api,2);
    readThread->createThread();

    ASSERT_TRUE(read_config_file());
    //
    // Set broadcast frequencies to their default values
    ack = api->setBroadcastFreqDefaults(wait_timeout);
    ASSERT_EQ(ack, ACK_SUCCESS);
  }

  // Code here will be called immediately after 
  // each test but before the destructor
  virtual void TearDown()
  {
    // Reset broadcast frequencies back to their default values
    ack = api->setBroadcastFreqDefaults(wait_timeout);
    ASSERT_EQ(ack, ACK_SUCCESS);
  }

  ::testing::AssertionResult read ();

  void setControlStandard()
  {
  ack = api->setControl(true, wait_timeout);

  if(ack == ACK_SETCONTROL_OBTAIN_RUNNING)
    ack = api->setControl(true, wait_timeout);

  ASSERT_EQ(ack, ACK_SETCONTROL_OBTAIN_SUCCESS);
  }

  void releaseControlStandard()
  {
  ack = api->setControl(false, wait_timeout);

  if(ack == ACK_SETCONTROL_RELEASE_RUNNING)
    ack = api->setControl(false, wait_timeout);

  ASSERT_EQ(ack, ACK_SETCONTROL_RELEASE_SUCCESS);
  }


  /*
   * Authentication standard method 
   */
  void activateDroneStandard()
  {
    char app_key[65];
    user_ack_data.encKey = app_key; 
    strcpy(user_ack_data.encKey, enc_key.c_str());
    user_ack_data.ID = app_id;
    Version targetVersion = versionM100_31;
    api->setVersion(targetVersion);
    ack = api->activate(&user_ack_data, wait_timeout);
    ASSERT_EQ(ack, ACK_ACTIVE_SUCCESS);
  }


   /*
   * Authentication method with params 
   */
  unsigned short activateDroneStandard(int l_app_id, std::string l_enc_key, Version version)
  {
    ActivateData testActivateData; 
    char app_key[65];
    testActivateData.encKey = app_key; 
    strcpy(testActivateData.encKey, l_enc_key.c_str());
    testActivateData.ID = l_app_id;
    api->setVersion(version);
    return api->activate(&testActivateData, wait_timeout);
  }

  ::testing::AssertionResult read_config_file()
  {
    // From stackoverflow - Maik Beckmann
    std::ifstream ifs("../config.json");
    std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    std::string err;
    auto config = json11::Json::parse(content, err);
    if(config != nullptr)
    {
      // targetVersion = config["target_version"].string_value();
      app_id = config["api_id"].int_value();
      enc_key = config["api_key"].string_value();
      return ::testing::AssertionSuccess();
    } 
    return ::testing::AssertionFailure() << err;
  }

  void std_sleep()
  {
    sleep(30);
  }

  CoreAPI *api;
  ActivateData user_ack_data;
  Version targetVersion;
  BroadcastData broadcastData;
  VersionData versionData;
  APIThread *readThread;

  // Activation data
  int app_id;
  std::string enc_key;

  /*
   * Test data
   */

  // TODO implement high resolution timeout
  int wait_timeout = 10;
  unsigned short ack;

  // TODO: implement hardware abstraction layer
  # if(WIN32)
    QHardDriver *serialDevice;
  # else 
    HardDriver_Manifold *serialDevice;
  #endif

  std::string sDevice;
  unsigned int baudrate;

};

class Timer
{
  private:
    clock_t startTime;
  public:
    void start ()
    {
      startTime = clock ();
    }

    unsigned long elapsedTime ()
    { 
      return (clock() - startTime) / CLOCKS_PER_SEC;
    }

    bool isTimeout (double seconds)
    {
      return seconds >= elapsedTime ();
    }
};
