#include "testing.hpp"

// import flight control sample codes
#include "flight_sample.hpp"
#include "dji_linux_helpers.hpp"
#include "fuzzer.hpp"
#include <random>
#include <typeinfo>

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

  // test random generation

  std::default_random_engine generator;

  std::uniform_real_distribution<double> continuousDistribution = std::uniform_real_distribution<double>(2.0,5.0);
  std::cout << typeid(continuousDistribution(generator)).name() << endl;
  std::cout << typeid((float32_t) continuousDistribution(generator)).name() << endl;
    for (int i=0; i<20; ++i) {
    std::cout << (float32_t) continuousDistribution(generator) << ", ";
  }
  std::cout << endl;

  Fuzzer fuzzer = Fuzzer();
  fuzzer.initializeModeGrammar();  
  fuzzer.initializeCommandGrammar();

  FlightController::JoystickMode joystickMode = fuzzer.generateModeWithGrammar();
  // std::cout << joystickMode.horizontalCoordinate << joystickMode.horizontalLogic << joystickMode.verticalLogic << joystickMode.yawLogic << joystickMode.stableMode << endl;

  DJI::OSDK::FlightController::JoystickCommand joystickCommand = fuzzer.generateCommandWithGrammar(joystickMode);

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
