#include <cstdlib>
#include <iostream>
#include <map>
#include <list>
#include <string>
#include <vector>
#include "fuzzer.hpp"

#include "dji_flight_joystick_module.hpp" // for enum values
#include "dji_flight_controller.hpp" // for mode struct

using namespace std;

Fuzzer::Fuzzer(){
  srand(time(NULL));
}

void Fuzzer::initializeModeGrammar(){
  Fuzzer::modeGrammar["horizFrame"] = {
    DJI::OSDK::FlightJoystick::HorizontalCoordinate::HORIZONTAL_BODY,
    DJI::OSDK::FlightJoystick::HorizontalCoordinate::HORIZONTAL_GROUND
  };
  Fuzzer::modeGrammar["horizMode"] = {
    DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_ANGLE,
    DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_ANGULAR_RATE,
    DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_POSITION,
    DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_VELOCITY
  };
  Fuzzer::modeGrammar["stableMode"] = {
    DJI::OSDK::FlightJoystick::StableMode::STABLE_DISABLE,
    DJI::OSDK::FlightJoystick::StableMode::STABLE_ENABLE
  };
  Fuzzer::modeGrammar["vertiMode"] = {
    DJI::OSDK::FlightJoystick::VerticalLogic::VERTICAL_POSITION,
    DJI::OSDK::FlightJoystick::VerticalLogic::VERTICAL_THRUST,
    DJI::OSDK::FlightJoystick::VerticalLogic::VERTICAL_VELOCITY
  };
  Fuzzer::modeGrammar["yawMode"] = {
    DJI::OSDK::FlightJoystick::YawLogic::YAW_ANGLE,
    DJI::OSDK::FlightJoystick::YawLogic::YAW_RATE
  };
}

DJI::OSDK::FlightController::JoystickMode Fuzzer::generateModeWithGrammar() {

    DJI::OSDK::FlightController::JoystickMode joystickMode = {
      // enums are stored as integers, so need to recast them before composing the new instance
      (DJI::OSDK::FlightJoystick::HorizontalLogic) Fuzzer::modeGrammar["horizMode"][rand() % Fuzzer::modeGrammar["horizMode"].size()],
      (DJI::OSDK::FlightJoystick::VerticalLogic) Fuzzer::modeGrammar["vertiMode"][rand() % Fuzzer::modeGrammar["vertiMode"].size()],
      (DJI::OSDK::FlightJoystick::YawLogic) Fuzzer::modeGrammar["yawMode"][rand() % Fuzzer::modeGrammar["yawMode"].size()],
      (DJI::OSDK::FlightJoystick::HorizontalCoordinate) Fuzzer::modeGrammar["horizFrame"][rand() % Fuzzer::modeGrammar["horizFrame"].size()],
      (DJI::OSDK::FlightJoystick::StableMode) Fuzzer::modeGrammar["stableMode"][rand() % Fuzzer::modeGrammar["stableMode"].size()]
    };

    return joystickMode;
}

// randomize mode bit-level


// randomize command values


// randomize command bit-level