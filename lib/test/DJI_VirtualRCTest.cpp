#include "DJI_VirtualRCTest.h"


TEST_F(DJI_VirtualRCTest, isVirtualRC)
{
  SKIP_TEST();
  virtualrc->isVirtualRC();
  EXPECT_TRUE(read ());
}

TEST_F(DJI_VirtualRCTest, getVRCData)
{
  SKIP_TEST();
  virtualrc->getVRCData();
  // should have an assert that checks for whether it matches some predefined values
  // by setting those values... somehow
  EXPECT_TRUE(read ());
}
