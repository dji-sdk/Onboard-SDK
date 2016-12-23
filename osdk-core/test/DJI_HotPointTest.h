#ifndef ONBOARDSDK_DJI_HOTPOINTTEST_H
#define ONBOARDSDK_DJI_HOTPOINTTEST_H

#include "DJI_FlightTest.h"

class DJI_HotPointTest : public DJI_FlightTest {
 protected:
  virtual void SetUp();
  virtual void TearDown();
  HotPoint *hotpoint;
  uint8_t hp_ack;
};

#endif
