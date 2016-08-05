#include "DJI_FlightTest.h"
#include <iostream> 

TEST_F(DJI_FlightTest, arm)
{
  SKIP_TEST();
  ack = flight->setArm(true, wait_timeout);
  ASSERT_EQ(ack, ACK_ARM_SUCCESS);
}

TEST_F(DJI_FlightTest, disarm)
{
  SKIP_TEST();
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
TEST_F(DJI_FlightTest, TakeOffLandingFlight)
{
  SKIP_TEST();
  ack = flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);
 
  std_sleep(); 

  ack = flight->task(DJI::onboardSDK::Flight::TASK_LANDING, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);

}

TEST_F(DJI_FlightTest, GoHomeNoMove)
{
  SKIP_TEST();
  std_sleep();
  ack = flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);
  std_sleep();
  ack = flight->task(DJI::onboardSDK::Flight::TASK_GOHOME, wait_timeout);
  ASSERT_EQ(ack, TASK_SUCCESS);
}

