#ifndef ONBOARDSDK_DJI_HOTPOINTTEST_H
#define ONBOARDSDK_DJI_HOTPOINTTEST_H

#include "DJI_APITest.h"

class DJI_HotPointTest : public DJI_APITest {
 protected:
  virtual void SetUp();
  virtual void TearDown();
  HotPoint *hotpoint;
  Flight *flight;
  uint8_t hp_ack;
};

#endif
