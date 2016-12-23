#ifndef ONBOARDSDK_DJI_WAYPOINTTEST_H
#define ONBOARDSDK_DJI_WAYPOINTTEST_H

#include "DJI_APITest.h"
#include "DJI_FlightTest.h"
#include <vector>
#include <stdexcept>

class DJI_WayPointTest : public DJI_FlightTest {
 protected:
  virtual void SetUp();
  virtual void TearDown();
  void set_waypoint_init_defaults(WayPointInitData* fdata);
  void set_waypoint_defaults(WayPointData* wp);
  void run_init_provide_test(int init_count, int prov_count);
  void generate_waypoints(WayPointData* start_data, float64_t increment, int num_wp);
  WayPointData create_start_waypoint(BroadcastData* prior_data, float32_t start_alt);
  void upload_waypoints(float64_t increment, float32_t start_alt, int wp_count);

  struct expected_wp_acks {
    uint8_t init_ack;
    uint8_t* upload_acks;
    uint8_t start_ack;
    uint8_t pause_ack;
  } e_wp_acks;

  WayPoint* waypoint;
  uint8_t wp_ack;

  std::vector<WayPointData> wp_list;
};

#endif
