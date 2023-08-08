#include "testing.hpp"

// import flight control sample codes
#include "flight_control_sample.hpp"
#include "flight_sample.hpp"
#include "dji_linux_helpers.hpp"

#include "grammar.hpp"

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

    flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){0, 0, 5.0}, 0, 2000);
    DSTATUS("Step 1 over!EmergencyBrake for 2s\n");
    flightSample->emergencyBrake();
    sleep(2);
    flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){-1.5, 2, 0}, 0, 2000);
    DSTATUS("Step 2 over!EmergencyBrakefor 2s\n");
    flightSample->emergencyBrake();
    sleep(2);
    flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){3, 0, 0}, 0, 2500);
    DSTATUS("Step 3 over!EmergencyBrake for 2s\n");
    flightSample->emergencyBrake();
    sleep(2);
    flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){-1.6, -2, 0}, 0, 2200);
    DSTATUS("Step 4 over!EmergencyBrake for 2s\n");
    flightSample->emergencyBrake();
    sleep(2);

    flightSample->monitoredLanding();

}


int main(int argc, char** argv)
{
  randomHorizMode();

  // srand(time(NULL));
  // cout << "Random pick: " << someArr[rand() % 3] << endl;
  // cout << "Map entries: " << enumValues["test"] << endl;
  // cout << "Map entries: " << enumValues["chungus"] << endl;
  // // for (int i = 0; i < enumValues["vertiMode"].size(); i++){
  // //   cout << "entry " << i << ": " << enumValues["vertiMode"][i] << endl;
  // for (const auto &entry: enumValues){
  //   cout << "{" << entry.first << ", " << entry.second << "}" << endl;
  // }

  

  // // 1. Setup OSDK.
  // LinuxSetup linuxEnvironment(argc, argv);

  // // 2. Setup Environment, 3. Initialize Communication, 4. Activate Vehicle
  // Vehicle* vehicle = linuxEnvironment.getVehicle();
  // if (vehicle == NULL)
  // {
  //   std::cout << "Vehicle not initialized, exiting.\n";
  //   return -1;
  // }

  // // Obtain Authority
  // vehicle->flightController->obtainJoystickCtrlAuthorityAsync(ObtainJoystickCtrlAuthorityCB, nullptr ,1, 2);

  // // Import flight functions
  // FlightSample* flightSample = new FlightSample(vehicle);
  
  // // Scenario A:
  // // flyOffset(flightSample);

  // // Scenario B:
  // flyVelocity(flightSample);
  
  return 0;
}
