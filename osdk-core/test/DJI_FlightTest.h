#ifndef ONBOARDSDK_DJI_FLIGHTTEST_H
#define ONBOARDSDK_DJI_FLIGHTTEST_H

#include "DJI_APITest.h"

class DJI_FlightTest : public DJI_APITest {
 protected:
  virtual void SetUp();
  virtual void TearDown();

  Flight *flight;
  unsigned short ack;
};

#endif
