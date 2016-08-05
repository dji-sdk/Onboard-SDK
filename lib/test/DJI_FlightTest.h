#include "DJI_APITest.h"

class DJI_FlightTest : public DJI_APITest
{
protected:
  virtual void SetUp()
  {
    // Set up the super fixture
    DJI_APITest::SetUp();

    activateDroneStandard();
    setControlStandard();
    flight = new Flight(DJI_APITest::api);

  }

  virtual void TearDown()
  {
    // Set up the super fixture
    DJI_APITest::TearDown();
    releaseControlStandard();
    delete flight;

  }

  Flight *flight;
  unsigned short ack;
};
