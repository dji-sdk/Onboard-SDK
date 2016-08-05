#include "DJI_APITest.h"

class DJI_HotPointTest : public DJI_APITest 
{
protected:
  virtual void SetUp()
  {
    // Set up the super fixture
    DJI_APITest::SetUp();
    activateDroneStandard();
    setControlStandard();
    hotpoint = new HotPoint(DJI_APITest::api);
    flight = new Flight(DJI_APITest::api);

    hp_ack = flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
    ASSERT_EQ(hp_ack, TASK_SUCCESS);
    std_sleep();
  }

  virtual void TearDown()
  {
    // Set up the super fixture
    DJI_APITest::TearDown();

    // Land if not landed already
    if(flight->getStatus() == DJI::onboardSDK::Flight::STATUS_SKY_STANDBY) 
    { 
      ack = flight->task(DJI::onboardSDK::Flight::TASK_GOHOME, wait_timeout);
      std_sleep();
    }
    releaseControlStandard();
    delete hotpoint;
    delete flight;
  }
  
  HotPoint *hotpoint;
  Flight * flight;
  uint8_t hp_ack;
    
}; 

