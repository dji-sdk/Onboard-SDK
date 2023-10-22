#include "testing.hpp"

// import flight control sample codes
#include "flight_sample.hpp"
#include "dji_linux_helpers.hpp"
#include "fuzzer.hpp"
#include <random>
#include <typeinfo>
#include <chrono>
#include <iostream>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

// callback functions for asynchronously obtaining authority
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData)
{
  if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ObtainJoystickCtrlAuthoritySuccess)
  {
    DSTATUS("ObtainJoystickCtrlAuthoritySuccess");
  }
}
void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData)
{
  if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ReleaseJoystickCtrlAuthoritySuccess)
  {
    DSTATUS("ReleaseJoystickCtrlAuthoritySuccess");
  }
}

void flyOffset(FlightSample* flightSample){

  flightSample->monitoredTakeoff();

  DSTATUS("Take off over!\n");
  flightSample->moveByPositionOffset((FlightSample::Vector3f){0, 6, 6}, 30, 0.8, 1);
  DSTATUS("Step 1 over!\n");
  flightSample->moveByPositionOffset((FlightSample::Vector3f){6, 0, -3}, -30, 0.8, 1);
  DSTATUS("Step 2 over!\n");
  flightSample->moveByPositionOffset((FlightSample::Vector3f){-6, -6, 0}, 0, 0.8, 1);
  DSTATUS("Step 3 over!\n");
  flightSample->monitoredLanding();

}

void flyVelocity(FlightSample* flightSample){
    flightSample->monitoredTakeoff();
    DSTATUS("Take off over!\n");

    for (int i = 1; i <= 10; i++){
      
      std::cout << "Iter :" << i << std::endl;

      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){0, 0, 5.0}, 0, 2000);
      // DSTATUS("Step 1 over!EmergencyBrake for 2s\n");
      // flightSample->emergencyBrake();
      // sleep(2);
      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){-1.5, 2, 0}, 0, 2000);
      // DSTATUS("Step 2 over!EmergencyBrakefor 2s\n");
      // flightSample->emergencyBrake();
      // sleep(2);
      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){3, 0, 0}, 0, 2000);
      // DSTATUS("Step 3 over!EmergencyBrake for 2s\n");
      // flightSample->emergencyBrake();
      // sleep(2);
      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){-1.6, -2, 0}, 0, 2000);
      // DSTATUS("Step 4 over!EmergencyBrake for 2s\n");
      // flightSample->emergencyBrake();
      // sleep(2);

    }

    flightSample->monitoredLanding();

}

void startFuzzing(int fuzzSecs, FlightSample *flightSample) {
  flightSample->monitoredLanding();

  Fuzzer fuzzer = Fuzzer();
  fuzzer.initializeModeGrammar();  
  fuzzer.initializeCommandGrammar();
  int count = 1;
  chrono::milliseconds fuzzMs = chrono::milliseconds(fuzzSecs * 1000);

  flightSample->monitoredTakeoff();
  DSTATUS("Take off over!\n");
  flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){0, 0, 3.0}, 0, 2000);

  chrono::milliseconds start = chrono::duration_cast< chrono::milliseconds >(chrono::system_clock::now().time_since_epoch());
  chrono::milliseconds now = chrono::duration_cast< chrono::milliseconds >(chrono::system_clock::now().time_since_epoch());
  while (now - start < fuzzMs) {
    std::cout << "======================================================"<< endl;
    std::cout << "#" << count++ << ", Elapsed: " << (now - start).count()/1000 << " seconds."<< endl;
    // flightSample->fuzz(fuzzer, 2000);

    double x, y, t;
    cout << "X-velocity:";
    cin >> x;
    cout << endl;
    cout << "Y-velocity:";
    cin >> y;
    cout << endl;
    cout << "Time:";
    cin >> t;
    flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){x, y, 0}, 0, t);

    // flightSample->emergencyBrake();
    // sleep(2);
    now = chrono::duration_cast< chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());

    // check if not responding anymore
    // land and restart fuzz
  }

 
  flightSample->monitoredLanding();
}


int main(int argc, char** argv)
{
  // 1. Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);

  // 2. Setup Environment, 3. Initialize Communication, 4. Activate Vehicle
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Authority
  vehicle->flightController->obtainJoystickCtrlAuthorityAsync(ObtainJoystickCtrlAuthorityCB, nullptr ,1, 2);

  // Import flight functions
  FlightSample* flightSample = new FlightSample(vehicle);
  
  // Scenario A:
  // flyOffset(flightSample);

  // Scenario B:
  // flyVelocity(flightSample);

  // Fuzz
  startFuzzing(600, flightSample);
  
  flightSample->monitoredLanding();

  // Fuzzer fuzzer = Fuzzer();
  // fuzzer.initializeModeGrammar();  
  // fuzzer.initializeCommandGrammar();
  
  // FlightController::JoystickMode mode = fuzzer.generateModeWithGrammar();
  // fuzzer.generateCommandWithGrammar(mode);
  
  
  return 0;
}
