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
  } else if(environment->getVersion() == "versionA3_32" ||
      environment->getVersion().empty()) {
    targetVersion = versionA3_32;
  }else{
    FAIL() << "Invalid version: " << environment->getVersion();
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

  api->setVersion(targetVersion);
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

TEST_F(DJI_APITest, getDroneVersion) {
  versionData = api->getDroneVersion(wait_timeout);
  ASSERT_EQ(versionData.version_ack, 0);

  if(api->getSDKVersion() == versionM100_31){
    EXPECT_STREQ(versionData.version_name, M100_31_FW_VERSION);
  }else if(api->getSDKVersion() == versionA3_31) {
    EXPECT_STREQ(versionData.version_name, A3_31_FW_VERSION);
  }else if(api->getSDKVersion() == versionA3_32) {
    EXPECT_STREQ(versionData.version_name, A3_32_FW_VERSION);
  }

  API_LOG(api->serialDevice, STATUS_LOG, "version ack = %d\n", versionData.version_ack);

  if(api->getSDKVersion() != versionA3_32){
   API_LOG(api->serialDevice, STATUS_LOG, "version crc = 0x%X\n", versionData.version_crc);
   if ( api->getSDKVersion() != versionM100_23)
     API_LOG(api->serialDevice, STATUS_LOG, "version ID = %.11s\n", versionData.version_ID);
  }

  API_LOG(api->serialDevice, STATUS_LOG, "version name = %.32s\n", versionData.version_name);
}

/**
 * Preconditions:
 *
 * 1. "Enable API Control" box checked in the Assistant software
 * 2. Mode selection bar of the remote controller is placed at the:
 * F position for M100_31, A3_31 and P position for A3_32
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
 * 2. Mode selection bar of the remote controller is placed at the:
 * F position for M100_31, A3_31 and P position for A3_32
 *
 */
TEST_F(DJI_APITest, releaseControl) {
  activateDroneStandard();
  releaseControlStandard();
}

TEST_F(DJI_APITest, setBroadcastFreq_01){
  //! TODO add more functionality to test
  //! given frequency rate. At the moment, if
  //! you use test as it is you can use
  //! Assistant software to verify frequency
  //! setter.

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

  uint8_t freq[16];
  freq[0] = BROADCAST_FREQ_1HZ;
  freq[1] = BROADCAST_FREQ_10HZ;

  ack = api->setBroadcastFreq(freq, wait_timeout);
  ASSERT_EQ(ack, ACK_SUCCESS);
}

TEST_F(DJI_APITest, DISABLED_setSyncFreq) {}
