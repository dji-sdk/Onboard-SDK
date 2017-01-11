#include "DJI_FlightTest.h"

using namespace DJI::onboardSDK;

void DJI_FlightTest::SetUp() {
  DJI_APITest::SetUp();

  activateDroneStandard();
  setControlStandard();
  flight = new Flight(DJI_APITest::api);
  
  // Set defaults 
  ack = 0xFF;
  
  /*
   * Check battery status
   *
   * @note
   * If using Non-DJI battery, battery status is zero
   */
/*  BatteryData battery = api->getBroadcastData().battery;
  if(battery < BATTERY_MIN_CAPACITY)
    FAIL() << "Battery capacity is too low.";
*/
}

void DJI_FlightTest::TearDown() {
  releaseControlStandard();

  delete flight;
  DJI_APITest::TearDown();
}

void DJI_FlightTest::doTask(Flight::TASK task){
  ack = flight->task(task, 1);
  /*
   * In simulation mode, sometimes returns with TASK_SUCCESS
   * but does not take off
   */
  ASSERT_EQ(ack, TASK_SUCCESS);
}

int DJI_FlightTest::processTask() {
  FlightStatus RESULT_STATUS;
  Timer timer;
  double elapsed;

  RESULT_STATUS = (task == Flight::TASK_TAKEOFF) ?
      Flight::STATUS_SKY_STANDBY : Flight::STATUS_GROUND_STANDBY;

  // Wait for 15 seconds to task to process
  timer.start();
  do{
    sleep(1);
    elapsed = elapsed + timer.elapsed().count();
    if((elapsed >= TASK_TIMEOUT) && (task == Flight::TASK_TAKEOFF))
      return -1;

    status = flight->getStatus();
  }while(status != RESULT_STATUS);

  return 0;
}

float32_t DJI_FlightTest::waitToSyncHeight() {
  float32_t deltaHeight;
  float32_t currentHeight;
  float32_t delta;

  // Wait for BroadcastData to catch up with position values
  deltaHeight = getHeight();
  do{
    sleep(3);
    currentHeight = getHeight();
    delta = fabs(currentHeight - deltaHeight);
    deltaHeight = currentHeight;
  }while(delta >= 0.009);

  return currentHeight;
}

void DJI_FlightTest::takeOffWithRetry(){
  int result;
  int retry = 1;
  task = Flight::TASK_TAKEOFF;

  do{
    sleep(1);
    doTask(task);
    result = processTask();
    if(result == 0)
      break;
  }while(++retry < TAKEOFF_MAX_RETRY);

  if(result == 0)
    // The two float values are almost equal
    ASSERT_PRED_FORMAT2(::testing::FloatLE, waitToSyncHeight(), TAKE_OFF_MAX_HOVER_HEIGHT);
  else
    FAIL() << "Failed Task: TakeOFF";
}

float32_t DJI_FlightTest::getHeight() {
  if(DJI_APITest::targetVersion != versionM100_31)
    return DJI_APITest::api->getBroadcastData().pos.height;
  else
    return DJI_APITest::api->getBroadcastData().pos.altitude;
}

void DJI_FlightTest::land(Flight::TASK task) {
  doTask(task);
  if(processTask() == 0){
    // The two float values are almost equal
    ASSERT_PRED_FORMAT2(::testing::FloatLE, waitToSyncHeight(), LANDING_MAX_HEIGHT);
  }else{
    FAIL() << "Failed Task: landing";
  }
}

/**
 * If you want to test in non simulation mode, disconnect
 * from simulator and restart aircraft
 */
TEST_F(DJI_FlightTest, armFresh) {
  ack = flight->setArm(true, wait_timeout);
  ASSERT_EQ(ack, ACK_ARM_SUCCESS);
  sleep(4);
}

TEST_F(DJI_FlightTest, disarmFresh) {
  ack = flight->setArm(false, wait_timeout);
  if(!sim_control_enabled){
    ASSERT_EQ(ack, ACK_ARM_SUCCESS);
  }else{
    /*
     * @note
     * In simulator mode, ACK_ARM_ALREADY_ARMED 
     * returns if an aircraft is already in requested 
     * ARM state
     */ 
    ASSERT_EQ(ack, ACK_ARM_ALREADY_ARMED);
  }
}

TEST_F(DJI_FlightTest, armDisarm) {
  ack = flight->setArm(true, wait_timeout);
  ASSERT_EQ(ack, ACK_ARM_SUCCESS);
  sleep(2);
  ack = flight->setArm(false, wait_timeout);
  ASSERT_EQ(ack, ACK_ARM_SUCCESS);
}

TEST_F(DJI_FlightTest, landFresh) {
  /*
   * In simulation mode, initial
   * altitude is 0.1
   */
  land(task = Flight::TASK_LANDING);
}

TEST_F(DJI_FlightTest, takeOFFLand) {
  takeOffWithRetry();
  land(task = Flight::TASK_LANDING);
}

TEST_F(DJI_FlightTest, takeOFFGoHome) {
  takeOffWithRetry();
  land(task = Flight::TASK_GOHOME);
}

