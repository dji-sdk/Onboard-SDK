#include "DJI_APITest.h"

using namespace DJI;
using namespace DJI::onboardSDK;

DJI_APITest::DJI_APITest() {}

DJI_APITest::~DJI_APITest() {}

void DJI_APITest::SetUp() {
  sDevice = environment->getDevice();
  baudrate = environment->getBaudrate();

  if (environment->getVersion() == "versionM100_31") {
    targetVersion = versionM100_31;
  } else if (environment->getVersion() == "versionA3_31") {
    targetVersion = versionA3_31;
  } else {
    std::cout << "Invalid version: " << environment->getVersion() << std::endl;
    FAIL();
  }

  app_id = environment->getApp_id();
  enc_key = environment->getEnc_key();
  sim_control_enabled = environment->isSim_control_enabled();
  if (sim_control_enabled) {
    sim_path = environment->getSim_path();
    sim_args =
        " " + environment->getSim_ip() + " " + environment->getSim_hash() + " ";
    sim_timeout_s = environment->getSim_timeout_s();
  }

#if WIN32
  serialDevice = new QHardDriver(new QSerialPort());
#else
  serialDevice = new LinuxSerialDevice(sDevice, baudrate);
#endif

  stop_simulator();
  start_simulator();

  serialDevice->init();

  ASSERT_TRUE(serialDevice->getDeviceStatus());

  api = new CoreAPI(serialDevice);

  readThread = new LinuxThread(api, 2);
  readThread->createThread();

  ack = api->setBroadcastFreqDefaults(wait_timeout);
  ASSERT_EQ(ack, ACK_SUCCESS);
}

void DJI_APITest::TearDown() {
  ack = api->setBroadcastFreqDefaults(wait_timeout);
  ASSERT_EQ(ack, ACK_SUCCESS);
  stop_simulator();

  readThread->stopThread();
  delete readThread;
  delete api;
  delete serialDevice;
}

::testing::AssertionResult DJI_APITest::read() {
  int read_len = 0;
  int last_read = 0;
  uint8_t buf[BUFFER_SIZE];

  while (true) {
    // End of ACK frame
    if (api->getACKFrameStatus() == 0) {
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
    read_len = serialDevice->readall(buf, BUFFER_SIZE);
    for (int i = 0; i < read_len; i++) {
      // TODO re-evaluate recv buffer handler
      api->byteHandler(buf[i]);
    }
  }
  return ::testing::AssertionFailure() << "No response from flight controller";
}

void DJI_APITest::start_simulator() {
  if (sim_control_enabled) {
    std::string cmd = sim_path + sim_args + "start";
    std::cout << cmd << std::endl << std::flush;
    ASSERT_EQ(0, system(cmd.c_str()));
    sleep(sim_timeout_s);
  }
}

void DJI_APITest::stop_simulator() {
  if (sim_control_enabled) {
    std::string cmd = sim_path + sim_args + "stop";
    std::cout << cmd << std::endl << std::flush;
    ASSERT_EQ(0, system(cmd.c_str()));
    sleep(sim_timeout_s);
  }
}

void DJI_APITest::setControlStandard() {
  ack = api->setControl(true, wait_timeout);

  if (ack == ACK_SETCONTROL_OBTAIN_RUNNING)
    ack = api->setControl(true, wait_timeout);

  ASSERT_EQ(ack, ACK_SETCONTROL_OBTAIN_SUCCESS);
}

void DJI_APITest::releaseControlStandard() {
  ack = api->setControl(false, wait_timeout);

  if (ack == ACK_SETCONTROL_RELEASE_RUNNING) {
    ack = api->setControl(false, wait_timeout);
  }

  ASSERT_EQ(ack, ACK_SETCONTROL_RELEASE_SUCCESS);
}

/*
 * Authentication standard method
 */
void DJI_APITest::activateDroneStandard() {
  char app_key[65];
  user_ack_data.encKey = app_key;
  strcpy(user_ack_data.encKey, enc_key.c_str());
  user_ack_data.ID = app_id;
  api->setVersion(targetVersion);
  ack = api->activate(&user_ack_data, wait_timeout);
  ASSERT_EQ(ack, ACK_ACTIVE_SUCCESS);
}

/*
* Authentication method with params
*/
unsigned short DJI_APITest::activateDroneStandard(int l_app_id,
                                                  std::string l_enc_key,
                                                  Version version) {
  ActivateData testActivateData;
  char app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;
  api->setVersion(version);
  return api->activate(&testActivateData, wait_timeout);
}

void DJI_APITest::std_sleep() { sleep(30); }

TEST_F(DJI_APITest, activate) {
  ack = activateDroneStandard(app_id, enc_key, targetVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_SUCCESS);
}

TEST_F(DJI_APITest, activate_invalid_version) {
  // Set invalid target version
  Version targetVersion = 230451;
  ack = activateDroneStandard(app_id, enc_key, targetVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_VERSION_ERROR);
}

TEST_F(DJI_APITest, activate_invalid_key) {
  int l_app_id = 1000000;
  std::string l_enc_key =
      "0000000000000000000000000000000000000000000000000000000000000000";
  ack = activateDroneStandard(l_app_id, l_enc_key, targetVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_NEW_DEVICE);
}

TEST_F(DJI_APITest, DISABLED_getDroneVersion) {
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
TEST_F(DJI_APITest, obtainControl) {
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
TEST_F(DJI_APITest, releaseControl) {
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

TEST_F(DJI_APITest, DISABLED_setSyncFreq) {}
