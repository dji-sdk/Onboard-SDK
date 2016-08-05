#include "DJI_APITest.h"

using namespace DJI;
using namespace DJI::onboardSDK;

//#if defined(__LINUX__)



::testing::AssertionResult DJI_APITest::read ()
{
  int read_len = 0;
  int last_read = 0;
  uint8_t buf[BUFFER_SIZE];

  while (true)
  {
	// End of ACK frame
    if (api->getACKFrameStatus() == 0)
    {
      //! Reset ACK status
      api->setACKFrameStatus(DEFAULT_STATUS);
      return ::testing::AssertionSuccess();
    }
/*
    // End of broadcast frame
    if(api->getBroadcastFrameStatus())
    {
      //! Reset broadcast frame status
	  api->setBroadcastFrameStatus(false);
	  return ::testing::AssertionSuccess();
    }
*/
    read_len = serialDevice->readall (buf, BUFFER_SIZE);
    for (int i = 0; i < read_len; i++)
    {
      // TODO re-evaluate recv buffer handler
      api->byteHandler (buf[i]);
    }
  }
  return ::testing::AssertionFailure() << "No response from flight controller";
}



TEST_F(DJI_APITest,activate)
{
  Version targetVersion = versionM100_31;
  ack = activateDroneStandard(app_id, enc_key, targetVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_SUCCESS);
}

TEST_F(DJI_APITest,activate_invalid_version)
{
  // Set invalid target version
  Version targetVersion = 230451;
  ack = activateDroneStandard(app_id, enc_key, targetVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_VERSION_ERROR);
}

TEST_F(DJI_APITest,activate_invalid_key)
{
  Version targetVersion = versionM100_31;
  int l_app_id = 1000000;
  std::string l_enc_key = "0000000000000000000000000000000000000000000000000000000000000000";
  ack = activateDroneStandard(l_app_id, l_enc_key, targetVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_NEW_DEVICE);
}

TEST_F(DJI_APITest, getDroneVersion)
{
  versionData = api->getDroneVersion(wait_timeout);
  ASSERT_EQ(versionData.version_ack, 0);
}

/**
 * Preconditions:
 *
 * 1. "Enable API Control" box checked in the Assistant software
 * 2. Mode selection bar of the remote controller is placed at the
 * F position
 *
 */
TEST_F(DJI_APITest, obtainControl)
{
  activateDroneStandard();
  setControlStandard();
}

/**
 * Preconditions:
 *
 * 1. "Enable API Control" box checked in the Assistant software
 * 2. Mode selection bar of the remote controller is placed at the
 * F position
 *
 */
TEST_F(DJI_APITest, releaseControl)
{
  activateDroneStandard();
  releaseControlStandard();
}

/*
TEST_F(DJI_APITest, setBroadcastFreq_01)
{
  //! TODO add more functionality to test
  //! given frequency rate. At the moment, if
  //! you use test as it is you can use
  //! Assistant software to verify frequency
  //! setter.

  //ASSERT_TRUE(m_hd->getDevieStatus());

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
/*
  uint8_t freq[2];
  freq[0] = BROADCAST_FREQ_1HZ;
  freq[1] = BROADCAST_FREQ_10HZ;

  ack = api->setBroadcastFreq(freq, wait_timeout);
  ASSERT_EQ(ack, ACK_SUCCESS);
}
*/

TEST_F(DJI_APITest, DISABLED_setSyncFreq)
{
  //TODO implement feature
}

//#endif

int main (int argc, char** argv)
{
  /**
   * Throw an exception on failure, which will be interpreted
   * by testing framework as a test failure.
   */
  //testing::GTEST_FLAG(throw_on_failure) = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}
