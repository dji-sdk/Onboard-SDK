#include "DJI_FlightTest.h"

void DJI_FlightTest::SetUp() {
  DJI_APITest::SetUp();

  activateDroneStandard();
  setControlStandard();
  flight = new Flight(DJI_APITest::api);
}

void DJI_FlightTest::TearDown() {
  releaseControlStandard();
  delete flight;

  DJI_APITest::TearDown();
}

TEST_F(DJI_FlightTest, DISABLED_arm) {
  ack = flight->setArm(true, wait_timeout);
  ASSERT_EQ(ack, ACK_ARM_SUCCESS);
}

TEST_F(DJI_FlightTest, DISABLED_disarm) {
  ack = flight->setArm(false, wait_timeout);
  ASSERT_EQ(ack, ACK_ARM_SUCCESS);
}

/*
 * Take off and Landing tests
 * Should also have a test with taking off twice,
 * landing twice,
 * landing before taking off,
 * and other perms.
 */
TEST_F(DJI_FlightTest, DISABLED_TakeOffLandingFlight) {
  ack = flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);

  std_sleep();

  ack = flight->task(DJI::onboardSDK::Flight::TASK_LANDING, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);
}

TEST_F(DJI_FlightTest, DISABLED_GoHomeNoMove) {
  std_sleep();
  ack = flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);
  std_sleep();
  ack = flight->task(DJI::onboardSDK::Flight::TASK_GOHOME, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);
}
