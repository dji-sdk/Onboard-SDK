#include "DJI_HotPointTest.h"

void DJI_HotPointTest::SetUp() {
  DJI_APITest::SetUp();

  activateDroneStandard();
  setControlStandard();
  hotpoint = new HotPoint(DJI_APITest::api);
  flight = new Flight(DJI_APITest::api);

  hp_ack = flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  ASSERT_EQ(hp_ack, TASK_SUCCESS);
  std_sleep();
}

void DJI_HotPointTest::TearDown() {
  // Land if not landed already
  if (flight->getStatus() == DJI::onboardSDK::Flight::STATUS_SKY_STANDBY) {
    ack = flight->task(DJI::onboardSDK::Flight::TASK_GOHOME, wait_timeout);
    std_sleep();
  }
  releaseControlStandard();
  delete hotpoint;
  delete flight;

  DJI_APITest::TearDown();
}

TEST_F(DJI_HotPointTest, HotPointTest) {
  hotpoint->initData();
  hotpoint->setHotPoint(api->getBroadcastData().pos.longitude,
                        api->getBroadcastData().pos.latitude, 10);
  HotPointStartACK s = (hotpoint->start(wait_timeout));
  hp_ack = s.ack;
  // hp_ack = (hotpoint->start(wait_timeout)).ack;

  api->decodeMissionStatus(hp_ack);
  EXPECT_EQ(hp_ack, 0x00);
  std_sleep();
  std_sleep();
  std_sleep();
  hp_ack = hotpoint->stop(wait_timeout);
  api->decodeMissionStatus(hp_ack);
  EXPECT_EQ(hp_ack, 0x00);
}
