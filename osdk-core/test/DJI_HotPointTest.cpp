#include "DJI_HotPointTest.h"

void DJI_HotPointTest::SetUp() {
  DJI_FlightTest::SetUp();
  hotpoint = new HotPoint(DJI_APITest::api);

  FlightStatus status = DJI_FlightTest::flight->getStatus();

  //! @note Landing and Finishing landing is not exist anymore
  //  if (status == Flight::STATUS_LANDING ||
  //      status == Flight::STATUS_FINISHING_LANDING)
  //    waitToSyncHeight();
  //! @note Take off Flight::Status is not exist anymore
  //  else
  if (api->getFwVersion() > MAKE_VERSION(3,2,0,0)) {
    if (status == Flight::STATUS_SKY_STANDBY )
      land(task = Flight::TASK_GOHOME);
  } else { //! M100 Compatibility
    if (status == Flight::STATUS_SKY_STANDBY_M100ONLY || status == Flight::STATUS_TAKE_OFF_M100ONLY)
      land(task = Flight::TASK_GOHOME);
  }
}

void DJI_HotPointTest::TearDown() {
  delete hotpoint;
  DJI_FlightTest::TearDown();
}

TEST_F(DJI_HotPointTest, HotPointTest) {
  takeOffWithRetry();

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

  land(task = Flight::TASK_GOHOME);
}
