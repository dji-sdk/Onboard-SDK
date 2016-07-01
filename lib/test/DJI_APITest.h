#include <gtest/gtest.h>
#include "DJI_API.h"
#include "DJI_WayPoint.h"
#include "DJI_Flight.h"
#include "DJI_HardDriver_Manifold.h"
#include <ctime>
#include <time.h>
#include <unistd.h>

using namespace DJI::onboardSDK;

#define BUFFER_SIZE 1024
#define DEFAULT_STATUS 11

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

class DJI_APITest : public ::testing::Test
{
public:
  DJI_APITest ()
  {
    // TODO move device and baudrate initialization
    // to HW specific serial device implementation
    //device = "/dev/ttyTHS1";
    sDevice = "/dev/ttyUSB0";
    baudrate = 230400;

    serialDevice = new HardDriver_Manifold (sDevice, baudrate);
    serialDevice->init ();

    api = new CoreAPI ((HardDriver*) serialDevice);
    waypoint = new WayPoint(api);
    flight = new Flight(api);
  }

  //virtual ~DJI_APITest();

  // Code here will be called immediately
  // after the constructor but before each test
  virtual void SetUP ()
  {
    // TODO
  }

  // Code here will be called immediately after 
  // each test but before the destructor
  virtual void TearDown ()
  {
    // TODO
  }

  ::testing::AssertionResult read ();

  CoreAPI *api;
  ActivateData user_act_data;
  BroadcastData broadcastData;
  WayPoint *waypoint;
  Flight *flight;

  // TODO: implement hardware abstraction layer
  HardDriver_Manifold *serialDevice;
private:
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

