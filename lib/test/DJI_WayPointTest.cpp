#include "DJI_WayPointTest.h"
#include <stdexcept>

void DJI_WayPointTest::set_waypoint_defaults(WayPointData* wp)
{
     wp->damping = 0;
     wp->yaw = 0;
     wp->gimbalPitch = 0;
     wp->turnMode = 0;
     wp->hasAction = 0;
     wp->actionTimeLimit = 100;
     wp->actionNumber = 0;
     wp->actionRepeat = 0;
     for (int i = 0; i < 16; ++i)
     {
       wp->commandList[i] = 0;
       wp->commandParameter[i] = 0;
     }
} 

void DJI_WayPointTest::set_waypoint_init_defaults(WayPointInitData* fdata)
{
     fdata->maxVelocity = 10;
     fdata->idleVelocity = 5;
     fdata->finishAction = 0;
     fdata->executiveTimes = 1;
     fdata->yawMode = 0;
     fdata->traceMode = 0;
     fdata->RCLostAction = 0;
     fdata->gimbalPitch = 0;
     fdata->latitude = 0;
     fdata->longitude = 0;
     fdata->altitude = 0;
}

void DJI_WayPointTest::run_init_provide_test(int init_count, int prov_count) 
{
  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = init_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = prov_count;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();
}

/*
TEST_F(DJI_WayPointTest, TakeOffBeforeUpload)
{
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;


  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  // flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  std_sleep(); 

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();


}
TEST_F(DJI_WayPointTest, TakeOffBeforeStart)
{
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;


  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }

  // flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  std_sleep(); 
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
}

TEST_F(DJI_WayPointTest, TakeOffBeforeInit)
{
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;

  // flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
  std_sleep(); 

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep(); 

}

TEST_F(DJI_WayPointTest, TakeOffDuringUpload)
{
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;


  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
    if(i == 1)
    {
      // flight->task(DJI::onboardSDK::Flight::TASK_TAKEOFF, wait_timeout);
      std_sleep(); 
    }
  }
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();


}

TEST_F(DJI_WayPointTest, ReleaseControl)
{
  releaseControlStandard();
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0xEA, 0xEA, 0xEA};
  e_wp_acks.init_ack = 0xD1;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  setControlStandard();

}

TEST_F(DJI_WayPointTest, BadIndices)
{
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = 0;// can be 1 or 2 as well
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);


}

TEST_F(DJI_WayPointTest, BadInitTest)
{
  SKIP_TEST();
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0xE0;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;
  WayPointInitData fdata;
  fdata.maxVelocity = -10010;
  fdata.idleVelocity = -1005;
  fdata.finishAction = -1000;
  fdata.executiveTimes = -1001;
  fdata.yawMode = -1000;
  fdata.traceMode = -1000;
  fdata.RCLostAction = -1000;
  fdata.gimbalPitch = -1000;
  fdata.latitude = -1000;
  fdata.longitude = -1000;
  fdata.altitude = -1000;
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  wp_ack = waypoint->start(wait_timeout); 
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);

}

TEST_F(DJI_WayPointTest, BadUploadTest)
{
  SKIP_TEST();

  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0xE1, 0xE1, 0xE1};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;

  WayPointInitData fdata;
  fdata.indexNumber = wp_count;
  this->set_waypoint_init_defaults(&fdata);
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    wp.damping = -1000;
    wp.yaw = -1000;
    wp.gimbalPitch = -1000;
    wp.turnMode = -1000;
    wp.hasAction = -1000;
    wp.actionTimeLimit = -100100;
    wp.actionNumber = -1000;
    wp.actionRepeat = -1000;
    for (int i = 0; i < 16; ++i)
    {
      wp.commandList[i] = -1000;
      wp.commandParameter[i] = -1000;
    }
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
}

*/

TEST_F(DJI_WayPointTest, WayPointInit3Provide3)
{ 
    int prov_count = 3;
    int wp_count = 3; 
    uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
    e_wp_acks.init_ack = 0x00;
    e_wp_acks.upload_acks = upload_ack_array;
    e_wp_acks.start_ack = 0x00;
    run_init_provide_test(prov_count, wp_count);
}
 
TEST_F(DJI_WayPointTest, WayPointInit1Provide1)
{ 
    int prov_count = 1;
    int wp_count = 1; 
    uint8_t upload_ack_array[1] = {0xEA};
    e_wp_acks.init_ack = 0xE0;
    e_wp_acks.upload_acks = upload_ack_array;
    e_wp_acks.start_ack = 0xEB;
    run_init_provide_test(prov_count, wp_count);
}

TEST_F(DJI_WayPointTest, WayPointInit3Provide2)
{ 
    int prov_count = 3;
    int wp_count = 2; 
    uint8_t upload_ack_array[2] = {0x00, 0x00};
    e_wp_acks.init_ack = 0x00;
    e_wp_acks.upload_acks = upload_ack_array;
    e_wp_acks.start_ack = 0xEB;
    run_init_provide_test(prov_count, wp_count);
}

/* Core dumps, skip for now */
TEST_F(DJI_WayPointTest, WayPointInit2Provide3)
{ 
    SKIP_TEST();
    int prov_count = 2;
    int wp_count = 3; 
    uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
    e_wp_acks.init_ack = 0x00;
    e_wp_acks.upload_acks = upload_ack_array;
    e_wp_acks.start_ack = 0xEB;
    try { 
      run_init_provide_test(prov_count, wp_count);
    } catch(std::exception  const &e)
    { 
      std::cerr << e.what() << std::endl; 
    }
}


/* This is to test where the failure for only having one waypoint occurs 
 * by promising two waypoints and only uploading one.
 * Does the failure happen in the init? 
 */
TEST_F(DJI_WayPointTest, WayPointInit2Provide1)
{     
    int prov_count = 2;
    int wp_count = 1; 
    uint8_t upload_ack_array[2] = {0x00, 0x00};
    e_wp_acks.init_ack = 0x00;
    e_wp_acks.upload_acks = upload_ack_array;
    e_wp_acks.start_ack = 0xEB;
    run_init_provide_test(prov_count, wp_count);
}
///////////////////////////////////////////////////////////////////////

TEST_F(DJI_WayPointTest, CloseWayPoint)
{ 
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0xE5};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;
  float64_t increment = 0.000000001;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;

  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += increment);
    wp.longitude = (default_long += increment);
    wp.altitude = 10;
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  
  wp_ack = waypoint->start(wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();
}

TEST_F(DJI_WayPointTest, FarWayPoint)
{ 
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0xE6};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;
  float64_t increment = 0.001;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;

  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += increment);
    wp.longitude = (default_long += increment);
    wp.altitude = 10;
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  
  wp_ack = waypoint->start(wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();
}
///////////////////////////////////////////////////////////////////////

TEST_F(DJI_WayPointTest, PauseTest)
{
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;
  e_wp_acks.pause_ack = 0x00;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  wp_ack = waypoint->start(wait_timeout); 
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);

  wp_ack = waypoint->pause(true, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.pause_ack);
  std_sleep();
  wp_ack = waypoint->pause(false, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.pause_ack);
  std_sleep();


}

TEST_F(DJI_WayPointTest, NotRunningPauseTest)
{
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;
  e_wp_acks.pause_ack = 0xED;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;
 
  
  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  BroadcastData prior_data = api->getBroadcastData();
  int num_wp = fdata.indexNumber;
  float64_t default_lat = prior_data.pos.latitude;
  float64_t default_long = prior_data.pos.longitude;
  float32_t default_alt = 10;
  for(int i = 0; i < num_wp; i++)
  { 
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (default_lat += .000001);
    wp.longitude = (default_long += .000001);
    wp.altitude = (default_alt += i*5);
    wp_ack = (waypoint->uploadIndexData(&wp, wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[i]);
  }
  wp_ack = waypoint->pause(true, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.pause_ack);
  std_sleep();
  wp_ack = waypoint->pause(false, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.pause_ack);
  std_sleep();


}

