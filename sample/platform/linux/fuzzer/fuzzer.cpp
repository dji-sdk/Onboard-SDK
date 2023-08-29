#include <cstdlib>
#include <iostream>
#include <map>
#include <list>
#include <string>
#include <vector>
#include <random>
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

void Fuzzer::initializeCommandGrammar(){

  // for x and y
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_ANGLE] = {0, 35.0}; // degree 
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_VELOCITY] = {0, 30.0}; // m/s
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_POSITION] = {};
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::HorizontalLogic::HORIZONTAL_ANGULAR_RATE] = {0, 150.0}; // degree/s

  // z
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::VerticalLogic::VERTICAL_VELOCITY] = {-5.0, 5.0}; // m/s 
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::VerticalLogic::VERTICAL_POSITION] = {0, 120.0}; // m
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::VerticalLogic::VERTICAL_THRUST] = {0, 100.0}; // %

  // yaw
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::YawLogic::YAW_ANGLE] = {0, 360.0}; // self-inferred
  Fuzzer::cmdGrammar[DJI::OSDK::FlightJoystick::YawLogic::YAW_RATE] = {0, 150.0}; // degree/s

}

DJI::OSDK::FlightController::JoystickMode Fuzzer::generateModeWithGrammar() {

  random_device dev;
  default_random_engine generator(dev());

  std::cout << "Vector sizes: " << 
  Fuzzer::modeGrammar["horizMode"].size() << ", " <<
  Fuzzer::modeGrammar["vertiMode"].size() << ", " <<
  Fuzzer::modeGrammar["yawMode"].size() << ", " <<
  Fuzzer::modeGrammar["horizFrame"].size() << ", " <<
  Fuzzer::modeGrammar["stableMode"].size() << endl;

  std::cout << "Generated Random Mode positions: " << 
  std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizMode"].size()-1)(generator) << ", " <<
  std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["vertiMode"].size()-1)(generator) << ", " <<
  std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["yawMode"].size()-1)(generator) << ", " <<
  std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizFrame"].size()-1)(generator) << ", " <<
  std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["stableMode"].size()-1)(generator) << endl;

  DJI::OSDK::FlightController::JoystickMode joystickMode = {
    // enums are stored as integers, so need to recast them before composing the new instance
    (DJI::OSDK::FlightJoystick::HorizontalLogic) Fuzzer::modeGrammar["horizMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizMode"].size()-1)(generator)],
      
    (DJI::OSDK::FlightJoystick::VerticalLogic) Fuzzer::modeGrammar["vertiMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["vertiMode"].size()-1)(generator)],

    (DJI::OSDK::FlightJoystick::YawLogic) Fuzzer::modeGrammar["yawMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["yawMode"].size()-1)(generator)],

    (DJI::OSDK::FlightJoystick::HorizontalCoordinate) Fuzzer::modeGrammar["horizFrame"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizFrame"].size()-1)(generator)],

    (DJI::OSDK::FlightJoystick::StableMode) Fuzzer::modeGrammar["stableMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["stableMode"].size()-1)(generator)]
  };
  return joystickMode;
}

// randomize command grammar-based
DJI::OSDK::FlightController::JoystickCommand Fuzzer::generateCommandWithGrammar(DJI::OSDK::FlightController::JoystickMode mode) {

  random_device dev;
  default_random_engine generator(dev());

  DJI::OSDK::FlightController::JoystickCommand joystickCommand = {
    // x
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::HorizontalLogic) mode.horizontalLogic)][0], 
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::HorizontalLogic) mode.horizontalLogic)][1]
      )(generator),

    // y
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::HorizontalLogic) mode.horizontalLogic)][0], 
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::HorizontalLogic) mode.horizontalLogic)][1]
      )(generator),

    // z
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::VerticalLogic) mode.verticalLogic)][0], 
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::VerticalLogic) mode.verticalLogic)][1]
      )(generator),

    // yawRate
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::YawLogic) mode.yawLogic)][0], 
      Fuzzer::cmdGrammar[((DJI::OSDK::FlightJoystick::YawLogic) mode.yawLogic)][1]
      )(generator),
  };

  std::cout << "Command Range: " << Fuzzer::cmdGrammar[mode.horizontalLogic][0] << ", " << Fuzzer::cmdGrammar[mode.horizontalLogic][1] << ", Generated Command: " << joystickCommand.x << endl;
  std::cout << "Command Range: " << Fuzzer::cmdGrammar[mode.horizontalLogic][0] << ", " << Fuzzer::cmdGrammar[mode.horizontalLogic][1] << ", Generated Command: " << joystickCommand.y << endl;
  std::cout << "Command Range: " << Fuzzer::cmdGrammar[mode.verticalLogic][0] << ", " << Fuzzer::cmdGrammar[mode.verticalLogic][1] << ", Generated Command: " << joystickCommand.z << endl;
  std::cout << "Command Range: " << Fuzzer::cmdGrammar[mode.yawLogic][0] << ", " << Fuzzer::cmdGrammar[mode.yawLogic][1] << ", Generated Command: " << joystickCommand.yaw << endl;

  return joystickCommand;
}


// randomize command bit-level