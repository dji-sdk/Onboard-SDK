#include "DJI_APITest.h"

class DJI_VirtualRCTest : public DJI_APITest
{
protected:
  virtual void SetUp()
  {
    // Set up the super fixture
    DJI_APITest::SetUp();
    activateDroneStandard();
    setControlStandard();
  }

  virtual void TearDown()
  {
    // Set up the super fixture
    DJI_APITest::TearDown();
    releaseControlStandard();
  }

  VirtualRC *virtualrc;

};
