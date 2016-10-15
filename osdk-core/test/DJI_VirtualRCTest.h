#ifndef ONBOARDSDK_DJI_VIRTUALRCTEST_H
#define ONBOARDSDK_DJI_VIRTUALRCTEST_H

#include "DJI_APITest.h"

class DJI_VirtualRCTest : public DJI_APITest {
 protected:
  virtual void SetUp();
  virtual void TearDown();
  VirtualRC *virtualrc;
};

#endif
