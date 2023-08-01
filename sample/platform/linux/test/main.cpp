#include "testing.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{

  // 1. Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);

  // 2. Setup Environment, 3. Initialize Communication, 4. Activate Vehicle
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  
  // Subscribe to Data
  subscribeToDataAndSaveLogToFile(vehicle);
  
  return 0;
}
