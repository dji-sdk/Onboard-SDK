#include "DJI_WayPointTest.h"

void DJI_WayPointTest::SetUp() {
  DJI_FlightTest::SetUp();
  waypoint = new WayPoint(DJI_APITest::api);

  FlightStatus status = DJI_FlightTest::flight->getStatus();
  if (status == Flight::STATUS_LANDING ||
      status == Flight::STATUS_FINISHING_LANDING)
    waitToSyncHeight();
  else if (status == Flight::STATUS_SKY_STANDBY ||
      status == Flight::STATUS_TAKE_OFF)
    land(task = Flight::TASK_GOHOME);
}

void DJI_WayPointTest::TearDown() {
  delete waypoint;
  DJI_FlightTest::TearDown();
}

void DJI_WayPointTest::set_waypoint_defaults(WayPointData* wp) {
  wp->damping = 0;
  wp->yaw = 0;
  wp->gimbalPitch = 0;
  wp->turnMode = 0;
  wp->hasAction = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber = 0;
  wp->actionRepeat = 0;
  for (int i = 0; i < 16; ++i) {
    wp->commandList[i] = 0;
    wp->commandParameter[i] = 0;
  }
}

void DJI_WayPointTest::set_waypoint_init_defaults(WayPointInitData* fdata) {
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

void DJI_WayPointTest::run_init_provide_test(int init_count, int prov_count) {
  WayPointInitData fdata;

  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = init_count;
  float64_t increment = 0.000001;
  float32_t start_alt = 10;

  // Read WayPoint mission settings
  WayPointInitACK initACK;
  // Read WayPoint status at given index
  WayPointDataACK indexACK;

  // Some negative tests: No data uploaded yet
  initACK  = waypoint->getWaypointSettings(wait_timeout);
  // Waypoint settings not uploaded
  EXPECT_EQ(initACK.ack, 0xEA);

  indexACK = waypoint->getIndex(0, wait_timeout);
  // Waypoint index not uploaded
  EXPECT_EQ(indexACK.ack, 0xEB);

  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, prov_count);

  // Some positive tests
  initACK  = waypoint->getWaypointSettings(wait_timeout);
  indexACK = waypoint->getIndex(0, wait_timeout);

  // Must be more then one waypoint
  if(init_count > 1) {
    EXPECT_EQ(initACK.ack, 0x00);
    EXPECT_EQ(indexACK.ack, 0x00);
  } else {
    EXPECT_EQ(initACK.ack, 0xEA);
    EXPECT_EQ(indexACK.ack, 0xEB);
  }

  wp_ack = waypoint->start(wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();
}

void DJI_WayPointTest::generate_waypoints(WayPointData* start_data, float64_t increment, int num_wp) {
  // consider defining some sort of = operator for wp? for assertion test later
  for (int i = 0; i < num_wp; i++) {
    WayPointData wp;
    this->set_waypoint_defaults(&wp);
    wp.index = i;
    wp.latitude = (start_data->latitude + (increment*i));
    wp.longitude = (start_data->longitude + (increment*i));
    wp.altitude = (start_data->altitude + (5*i));
    wp_list.push_back(wp); 
  }
}

WayPointData DJI_WayPointTest::create_start_waypoint(BroadcastData* prior_data, float32_t start_alt) {
  WayPointData wp; 
  this->set_waypoint_defaults(&wp);
  wp.latitude = prior_data->pos.latitude;
  wp.longitude = prior_data->pos.longitude;
  wp.altitude = start_alt;
  return wp; 
}

void DJI_WayPointTest::upload_waypoints(float64_t increment, float32_t start_alt, int wp_count) {
  BroadcastData prior_data = api->getBroadcastData();
  WayPointData start_wp = this->create_start_waypoint(&prior_data, start_alt);

  this->generate_waypoints(&start_wp, increment, wp_count);
  for (std::vector<WayPointData>::iterator wp = wp_list.begin() ; wp != wp_list.end(); ++wp)
  {
    wp_ack = (waypoint->uploadIndexData(&(*wp), wait_timeout)).ack;
    api->decodeMissionStatus(wp_ack);
    EXPECT_EQ(wp_ack, e_wp_acks.upload_acks[(wp - wp_list.begin())]);
  }
}

TEST_F(DJI_WayPointTest, DISABLED_TakeOffBeforeUpload) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;

  int wp_count = 3;
  float64_t increment = .000001; 
  float32_t start_alt = 10; 


  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;


  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  takeOffWithRetry();

  upload_waypoints(increment, start_alt, wp_count);

  wp_ack = waypoint->start(wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();


}

TEST_F(DJI_WayPointTest, DISABLED_TakeOffBeforeStart) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;

  int wp_count = 3;
  float64_t increment = .000001; 
  float32_t start_alt = 10; 


  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;

  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, wp_count);

  // takeOffWithRetry();
  wp_ack = waypoint->start(wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
}

TEST_F(DJI_WayPointTest, DISABLED_TakeOffBeforeInit) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;

  int wp_count = 3;
  float64_t increment = .000001; 
  float32_t start_alt = 10; 

  // takeOffWithRetry();

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;


  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, wp_count);

  wp_ack = waypoint->start(wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();

}

TEST_F(DJI_WayPointTest, DISABLED_ReleaseControl) {
  releaseControlStandard();
  uint8_t upload_ack_array[3] = {0xEA, 0xEA, 0xEA};
  e_wp_acks.init_ack = 0xD1;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;

  int wp_count = 3;
  float64_t increment = .000001; 
  float32_t start_alt = 10; 

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;


  wp_ack = waypoint->init(&fdata, wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, wp_count);

  wp_ack = waypoint->start(wait_timeout);
    api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  setControlStandard();

}

TEST_F(DJI_WayPointTest, DISABLED_BadIndices) {
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
  for(int i = 0; i < num_wp; i++) {
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

TEST_F(DJI_WayPointTest, DISABLED_BadInitTest) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0xE0;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;

  int wp_count = 3;
  float64_t increment = .000001; 
  float32_t start_alt = 10; 

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

  upload_waypoints(increment, start_alt, wp_count);

  wp_ack = waypoint->start(wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);

}

TEST_F(DJI_WayPointTest, DISABLED_BadUploadTest) {
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
  for(int i = 0; i < num_wp; i++) {
    WayPointData wp;
    wp.damping = -1000;
    wp.yaw = -1000;
    wp.gimbalPitch = -1000;
    wp.turnMode = -1000;
    wp.hasAction = -1000;
    wp.actionTimeLimit = -100100;
    wp.actionNumber = -1000;
    wp.actionRepeat = -1000;
    for (int i = 0; i < 16; ++i) {
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

TEST_F(DJI_WayPointTest, WayPointInit3Provide3) {
  int prov_count = 3;
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;
  run_init_provide_test(prov_count, wp_count);
}

TEST_F(DJI_WayPointTest, WayPointInit1Provide1) {
  int prov_count = 1;
  int wp_count = 1;
  uint8_t upload_ack_array[1] = {0xEA};
  e_wp_acks.init_ack = 0xE0;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;
  run_init_provide_test(prov_count, wp_count);
}

TEST_F(DJI_WayPointTest, WayPointInit3Provide2) {
  int prov_count = 3;
  int wp_count = 2;
  uint8_t upload_ack_array[2] = {0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;
  run_init_provide_test(prov_count, wp_count);
}

/* Core dumps, skip for now */
TEST_F(DJI_WayPointTest, DISABLED_WayPointInit2Provide3) {
  int prov_count = 2;
  int wp_count = 3;
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;
  try {
    run_init_provide_test(prov_count, wp_count);
  } catch (std::exception const& e) {
    std::cerr << e.what() << std::endl;
  }
}

/* This is to test where the failure for only having one waypoint occurs
 * by promising two waypoints and only uploading one.
 * Does the failure happen in the init?
 */
TEST_F(DJI_WayPointTest, WayPointInit2Provide1) {
  int prov_count = 2;
  int wp_count = 1;
  uint8_t upload_ack_array[2] = {0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;
  run_init_provide_test(prov_count, wp_count);
}

TEST_F(DJI_WayPointTest, CloseWayPoint) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;

  float64_t increment = 0.000000001;
  int wp_count = 3;
  float32_t start_alt = 10;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;

  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, wp_count);

  wp_ack = waypoint->start(wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();
}

TEST_F(DJI_WayPointTest, DISABLED_FarWayPoint) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0xE6};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0xEB;

  int wp_count = 3;
  float64_t increment = 0.001;
  float32_t start_alt = 10;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;

  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, wp_count);

  wp_ack = waypoint->start(wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.start_ack);
  std_sleep();
}

TEST_F(DJI_WayPointTest, DISABLED_PauseTest) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;
  e_wp_acks.pause_ack = 0x00;

  int wp_count = 3;
  float32_t start_alt = 10;
  float64_t increment = 0.000001;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;

  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, wp_count);

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

  wp_ack = waypoint->stop(wait_timeout);
  api->decodeMissionStatus(wp_ack);
  if(wp_ack == 0xEC)
    std::cout << "Request is running\n";
  else
    EXPECT_EQ(wp_ack, 0x00);
}

TEST_F(DJI_WayPointTest, DISABLED_NotRunningPauseTest) {
  uint8_t upload_ack_array[3] = {0x00, 0x00, 0x00};
  e_wp_acks.init_ack = 0x00;
  e_wp_acks.upload_acks = upload_ack_array;
  e_wp_acks.start_ack = 0x00;
  e_wp_acks.pause_ack = 0xED;

  int wp_count = 3;
  float32_t start_alt = 10;
  float64_t increment = 0.000001;

  WayPointInitData fdata;
  this->set_waypoint_init_defaults(&fdata);
  fdata.indexNumber = wp_count;

  wp_ack = waypoint->init(&fdata, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.init_ack);

  upload_waypoints(increment, start_alt, wp_count);

  wp_ack = waypoint->pause(true, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.pause_ack);
  std_sleep();
  wp_ack = waypoint->pause(false, wait_timeout);
  api->decodeMissionStatus(wp_ack);
  EXPECT_EQ(wp_ack, e_wp_acks.pause_ack);
  std_sleep();
}
