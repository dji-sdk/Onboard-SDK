#include "DJI_APITest.h"

class DJI_WayPointTest : public DJI_APITest 
{
protected:
  virtual void SetUp()
  {
    // Set up the super fixture
    DJI_APITest::SetUp();
    activateDroneStandard();
    setControlStandard();
    waypoint = new WayPoint(DJI_APITest::api);
    flight = new Flight(DJI_APITest::api);
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
    delete waypoint;
    delete flight;
  }

  void set_waypoint_init_defaults(WayPointInitData* fdata);
  void set_waypoint_defaults(WayPointData* wp);
  void run_init_provide_test(int init_count, int prov_count);
 
  struct expected_wp_acks { 
    uint8_t init_ack;  
    uint8_t* upload_acks;  
    uint8_t start_ack;  
    uint8_t pause_ack;
  } e_wp_acks;
  
  WayPoint *waypoint;
  Flight * flight;
  uint8_t wp_ack;
    
}; 

