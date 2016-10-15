#include "DJI_VirtualRCTest.h"

void DJI_VirtualRCTest::SetUp() {
  DJI_APITest::SetUp();

  activateDroneStandard();
  setControlStandard();
}

void DJI_VirtualRCTest::TearDown() {
  releaseControlStandard();

  DJI_APITest::TearDown();
}

TEST_F(DJI_VirtualRCTest, DISABLED_isVirtualRC) {
  virtualrc->isVirtualRC();
  EXPECT_TRUE(read());
}

TEST_F(DJI_VirtualRCTest, DISABLED_getVRCData) {
  virtualrc->getVRCData();
  // should have an assert that checks for whether it matches some predefined
  // values
  // by setting those values... somehow
  EXPECT_TRUE(read());
}
