#ifndef ONBOARDSDK_DJI_FLIGHTTEST_H
#define ONBOARDSDK_DJI_FLIGHTTEST_H

#include "DJI_APITest.h"
#include "DJI_Utility.h"
#include <math.h>

using namespace DJI::onboardSDK;

/*
 * Aircraft will rise to 3.9 feet (1.18872m) and hover
 * above ground
 *
 * @note
 * Representation in simulation mode is in meters
 */
const float32_t TAKE_OFF_MAX_HOVER_HEIGHT = 1.2;
/*
 * In simulation mode, initial height is 0.1
 * Not in simulation mode, initial height is 0.00
 */
const float32_t LANDING_MAX_HEIGHT = 0.1;
/*
 * To perform a mission, battery must have at least
 * 50% of charge
 */
const uint8_t BATTERY_MIN_CAPACITY = 50;
/*
 * @note
 * Task timeout in milliseconds
 */
const double TASK_TIMEOUT = 15000;
const uint32_t TAKEOFF_MAX_RETRY = 3;

class DJI_FlightTest : public DJI_APITest {
 public:
  virtual void SetUp();
  virtual void TearDown();
 protected:
  Flight *flight;
  FlightStatus status;
  Flight::TASK task;
  unsigned short ack;

  void doTask(Flight::TASK task);
  int processTask();
  float32_t waitToSyncHeight();
  float32_t getHeight();
  void takeOffWithRetry();
  void land(Flight::TASK task);
};

#endif

