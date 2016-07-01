#include "DJI_APITest.h"

using namespace DJI;
using namespace DJI::onboardSDK;

#define SKIP_TEST() std::cout << "[  SKIPPED ] Feature " <<  "not supported" << std::endl; return;

::testing::AssertionResult DJI_APITest::read ()
{
  int read_len = 0;
  int last_read = 0;
  uint8_t buf[BUFFER_SIZE];

  while (true)
  {
    if (api->getSessionStatus() == 0)
    {
      //! Reset session status
      api->setSessionStatus(DEFAULT_STATUS);
      return ::testing::AssertionSuccess();
    }

    read_len = serialDevice->readall (buf, BUFFER_SIZE);
    for (int i = 0; i < read_len; i++)
    {
      // TODO re-evaluate recv buffer handler
      api->byteHandler (buf[i]);
    }
  }
  return ::testing::AssertionFailure() << "No response from flight controller";
}

TEST_F(DJI_APITest, getDroneVersion)
{
  ASSERT_TRUE(serialDevice->getDevieStatus());
  api->getDroneVersion ();
  EXPECT_TRUE(read ());
}

TEST_F(DJI_APITest,activate)
{
  SKIP_TEST();

  ASSERT_TRUE(serialDevice->getDevieStatus());

  int app_id = 1234;
  std::string enc_key = "abc";

  char app_key[65];
  user_act_data.encKey = app_key;
  strcpy(user_act_data.encKey, enc_key.c_str());

  user_act_data.ID = app_id;

  Version targetVersion = versionM100_31;
  api->setVersion(targetVersion);

  api->activate(&user_act_data);
  EXPECT_TRUE(read ());
}

TEST_F(DJI_APITest, setBroadcastFreq)
{
  SKIP_TEST();

  //! TODO add more functionality to test
  //! given frequency rate. At the moment, if
  //! you use test as it is you can use
  //! Assistant software to verify frequency
  //! setter.

  //ASSERT_TRUE(m_hd->getDevieStatus());

  uint8_t freq[12];

  /* BROADCAST_FREQ_0HZ
   * BROADCAST_FREQ_1HZ
   * BROADCAST_FREQ_10HZ
   * BROADCAST_FREQ_50HZ
   * BROADCAST_FREQ_100HZ
   */

  /* Channels definition:
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - Magnetometer
   * 7 - RC Channels Data
   * 8 - Gimbal Data
   * 9 - Flight Status
   * 10 - Battery Level
   * 11 - Control Information
   */

  freq[0] = BROADCAST_FREQ_50HZ;
  freq[1] = BROADCAST_FREQ_50HZ;
  freq[2] = BROADCAST_FREQ_10HZ;
  freq[3] = BROADCAST_FREQ_50HZ;
  freq[4] = BROADCAST_FREQ_10HZ;
  freq[5] = BROADCAST_FREQ_50HZ;
  freq[6] = BROADCAST_FREQ_10HZ;
  freq[7] = BROADCAST_FREQ_10HZ;
  freq[8] = BROADCAST_FREQ_100HZ;
  freq[9] = BROADCAST_FREQ_10HZ;
  freq[10] = BROADCAST_FREQ_50HZ;
  freq[11] = BROADCAST_FREQ_50HZ;

  api->setBroadcastFreq(freq);
  EXPECT_TRUE(read ());

  // TODO use timestamp to test the frequency
  // broadcastData = api->getBroadcastData();
  // broadcastData.timeStamp

}

TEST_F(DJI_APITest, setSyncFreq)
{
  SKIP_TEST();

  //TODO implement feature
}

//! This test demonstrates a simple step by step Waypoint mission execution
TEST_F(DJI_APITest, WaypointMission)
{
  ASSERT_TRUE(serialDevice->getDevieStatus());

  //! TODO Add your own App ID and Encryption Key
  int app_id = 1234;
  std::string enc_key = "abc";

  char app_key[65];
  user_act_data.encKey = app_key;
  strcpy(user_act_data.encKey, enc_key.c_str());

  user_act_data.ID = app_id;

  Version targetVersion = versionM100_31;
  api->setVersion(targetVersion);

  api->activate(&user_act_data);
  EXPECT_TRUE(read ());

  //! Obtain control
  api->setControl(true);

  //! Look for gear switch 
  int16_t gear_val = -4545; 
  std::cout << gear_val << std::endl;

  int16_t get_gear = api->getBroadcastData().rc.gear;

  while(get_gear == gear_val)
  {
    get_gear = api->getBroadcastData().rc.gear;
    EXPECT_TRUE(read ());

    std::cout << get_gear << std::endl;
    usleep(500000);
  }

  //! Takeoff
  flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF);
  usleep(10000000);

  WayPointInitData fdata;
  fdata.maxVelocity = 10;
  fdata.idleVelocity = 5;
  fdata.finishAction = 0;
  fdata.executiveTimes = 1;
  fdata.yawMode = 0;
  fdata.traceMode = 0;
  fdata.RCLostAction = 0;
  fdata.gimbalPitch = 0;
  fdata.latitude = 0;
  fdata.longitude = 0;
  fdata.altitude = 0;

  // Create two waypoints
  fdata.indexNumber = 0x3;

  waypoint->init(&fdata);
  EXPECT_TRUE(read ());

  WayPointData waypoint0;
  waypoint0.damping = 0;
  waypoint0.yaw = 0;
  waypoint0.gimbalPitch = 0;
  waypoint0.turnMode = 0;
  waypoint0.hasAction = 0;
  waypoint0.actionTimeLimit = 100;
  waypoint0.actionNumber = 0;
  waypoint0.actionRepeat = 0;

  for (int i = 0; i < 16; ++i)
  {
      waypoint0.commandList[i] = 0;
      waypoint0.commandParameter[i] = 0;
  }

  waypoint0.index = 0;
  waypoint0.latitude = 0.393446;
  waypoint0.longitude = 1.988958;
  waypoint0.altitude = 20;

  waypoint->uploadIndexData(&waypoint0);
  EXPECT_TRUE(read ());

  WayPointData waypoint1;
  waypoint1.damping = 0;
  waypoint1.yaw = 0;
  waypoint1.gimbalPitch = 0;
  waypoint1.turnMode = 0;
  waypoint1.hasAction = 0;
  waypoint1.actionTimeLimit = 100;
  waypoint1.actionNumber = 0;
  waypoint1.actionRepeat = 0;

  for (int i = 0; i < 16; ++i)
  {
     waypoint1.commandList[i] = 0;
     waypoint1.commandParameter[i] = 0;
  }

  waypoint1.index = 1;
  waypoint1.latitude = 0.393448;
  waypoint1.longitude = 1.988960;
  waypoint1.altitude = 40;

  waypoint->uploadIndexData(&waypoint1);
  EXPECT_TRUE(read ());


  WayPointData waypoint2;
  waypoint2.damping = 0;
  waypoint2.yaw = 0;
  waypoint2.gimbalPitch = 0;
  waypoint2.turnMode = 0;
  waypoint2.hasAction = 0;
  waypoint2.actionTimeLimit = 100;
  waypoint2.actionNumber = 0;
  waypoint2.actionRepeat = 0;

  for (int i = 0; i < 16; ++i)
  {
     waypoint2.commandList[i] = 0;
     waypoint2.commandParameter[i] = 0;
  }

  waypoint2.index = 2;
  waypoint2.latitude = 0.393446;
  waypoint2.longitude = 1.988958;
  waypoint2.altitude = 40;

  waypoint->uploadIndexData(&waypoint2);
  EXPECT_TRUE(read ());

  waypoint->start();
  usleep(100000000);
  EXPECT_TRUE(read ());

  //! If the aircraft does not take off, send explicit
  //! takeOff command and then start the Waypoint mission
  //flight->task((DJI::onboardSDK::Flight::TASK)takeOff);
  //EXPECT_TRUE(read ());

  //! Give it time to execute the command
  //sleep(100);

  //! Release control
  api->setControl(false);
  EXPECT_TRUE(read ());
}

int main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return RUN_ALL_TESTS ();
}
