#include <cstdlib>
#include <iostream>
#include <map>
#include <list>
#include <string>
#include <vector>
#include <random>
#include "fuzzer.hpp"
#include <boost/math/special_functions/round.hpp>

#include "dji_flight_joystick_module.hpp" // for enum values
#include "dji_flight_controller.hpp" // for mode struct

using namespace std;
using namespace DJI::OSDK;

Fuzzer::Fuzzer(){
  srand(time(NULL));
}

void Fuzzer::initializeModeGrammar(){
  Fuzzer::modeGrammar["horizFrame"] = {
    FlightJoystick::HorizontalCoordinate::HORIZONTAL_BODY,
    FlightJoystick::HorizontalCoordinate::HORIZONTAL_GROUND
  };
  Fuzzer::modeGrammar["horizMode"] = {
    FlightJoystick::HorizontalLogic::HORIZONTAL_ANGLE,
    FlightJoystick::HorizontalLogic::HORIZONTAL_ANGULAR_RATE,
    FlightJoystick::HorizontalLogic::HORIZONTAL_POSITION,
    FlightJoystick::HorizontalLogic::HORIZONTAL_VELOCITY
  };
  Fuzzer::modeGrammar["stableMode"] = {
    FlightJoystick::StableMode::STABLE_DISABLE,
    FlightJoystick::StableMode::STABLE_ENABLE
  };
  Fuzzer::modeGrammar["vertiMode"] = {
    FlightJoystick::VerticalLogic::VERTICAL_POSITION,
    FlightJoystick::VerticalLogic::VERTICAL_THRUST,
    FlightJoystick::VerticalLogic::VERTICAL_VELOCITY
  };
  Fuzzer::modeGrammar["yawMode"] = {
    FlightJoystick::YawLogic::YAW_ANGLE,
    FlightJoystick::YawLogic::YAW_RATE
  };
}

void Fuzzer::initializeCommandGrammar(){

  // for x and y
  vector<float> horizAngle = {0, 35.0};// degree
  vector<float> horizVelocity = {-2.0, 2.0};// m/s original is 30 m/s
  vector<float> horizPosition = {-10, 10}; // arbitrary
  vector<float> horizAngularRate = {-150, 150.0};// degree/s original is 150 deg/s

  map<int, vector<float>> horizontalGrammar;
  horizontalGrammar[FlightJoystick::HorizontalLogic::HORIZONTAL_ANGLE] = horizAngle;  
  horizontalGrammar[FlightJoystick::HorizontalLogic::HORIZONTAL_VELOCITY] = horizVelocity; 
  horizontalGrammar[FlightJoystick::HorizontalLogic::HORIZONTAL_POSITION] = horizPosition;
  horizontalGrammar[FlightJoystick::HorizontalLogic::HORIZONTAL_ANGULAR_RATE] = horizAngularRate; 
  Fuzzer::cmdGrammar["horizontalGrammar"] = horizontalGrammar;

  // z
  vector<float> vertiVelocity = {-1.0, 1.0};// m/s
  vector<float> vertiPosition = {0, 120.0};// m
  vector<float> vertiThrust = {0, 100.0}; // %

  map<int, vector<float>> vertiLogic;
  vertiLogic[FlightJoystick::VerticalLogic::VERTICAL_VELOCITY] = vertiVelocity; // m/s 
  vertiLogic[FlightJoystick::VerticalLogic::VERTICAL_POSITION] = vertiPosition; // m
  vertiLogic[FlightJoystick::VerticalLogic::VERTICAL_THRUST] = vertiThrust; // %
  Fuzzer::cmdGrammar["verticalGrammar"] = vertiLogic;

  // yaw
  vector<float> yawAngle = {0, 360.0};// self-infered
  vector<float> yawRate = {0, 150.0};// deg/s
  
  map<int, vector<float>> yawLogic;
  yawLogic[FlightJoystick::YawLogic::YAW_ANGLE] = yawAngle;
  yawLogic[FlightJoystick::YawLogic::YAW_RATE] = yawRate;
  Fuzzer::cmdGrammar["yawGrammar"] = yawLogic; 
  
  // for (auto const& [dir, grammar]: Fuzzer::cmdGrammar) {
  //   std::cout << "Direction: " << dir << std::endl;        // string (key)
  //   for (auto const& [key, val] : grammar) {
  //     std::cout << key        // string (key)
  //               << ':'  
  //               << val[0]        // string's value
  //               << ','  
  //               << val[1]        // string's value
  //               << std::endl;
  //   }
  // }
}

FlightController::JoystickMode Fuzzer::generateModeWithGrammar() {

  random_device dev;
  default_random_engine generator(dev());

  // std::cout << "Vector sizes: " << 
  // Fuzzer::modeGrammar["horizMode"].size() << ", " <<
  // Fuzzer::modeGrammar["vertiMode"].size() << ", " <<
  // Fuzzer::modeGrammar["yawMode"].size() << ", " <<
  // Fuzzer::modeGrammar["horizFrame"].size() << ", " <<
  // Fuzzer::modeGrammar["stableMode"].size() << endl;

  // std::cout << "Generated Random Mode positions: " << 
  // std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizMode"].size()-1)(generator) << ", " <<
  // std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["vertiMode"].size()-1)(generator) << ", " <<
  // std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["yawMode"].size()-1)(generator) << ", " <<
  // std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizFrame"].size()-1)(generator) << ", " <<
  // std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["stableMode"].size()-1)(generator) << endl;

  FlightController::JoystickMode joystickMode = {
    // enums are stored as integers, so need to recast them before composing the new instance
    (FlightJoystick::HorizontalLogic) Fuzzer::modeGrammar["horizMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizMode"].size()-1)(generator)],
      
    (FlightJoystick::VerticalLogic) Fuzzer::modeGrammar["vertiMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["vertiMode"].size()-1)(generator)],

    (FlightJoystick::YawLogic) Fuzzer::modeGrammar["yawMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["yawMode"].size()-1)(generator)],

    (FlightJoystick::HorizontalCoordinate) Fuzzer::modeGrammar["horizFrame"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["horizFrame"].size()-1)(generator)],

    (FlightJoystick::StableMode) Fuzzer::modeGrammar["stableMode"][
      std::uniform_int_distribution<int>(0, Fuzzer::modeGrammar["stableMode"].size()-1)(generator)]
  };

  return joystickMode;
}

// randomize command grammar-based
FlightController::JoystickCommand Fuzzer::generateCommandWithGrammar(FlightController::JoystickMode mode) {

  random_device dev;
  default_random_engine generator(dev());

  FlightController::JoystickCommand joystickCommand = {
    // x
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][0], 
      Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][1]
      )(generator),

    // y
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][0], 
      Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][1]
      )(generator),

    // z
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar["verticalGrammar"][mode.verticalLogic][0], 
      Fuzzer::cmdGrammar["verticalGrammar"][mode.verticalLogic][1]
      )(generator),

    // yawRate
    std::uniform_real_distribution<double>(
      Fuzzer::cmdGrammar["yawGrammar"][mode.yawLogic][0], 
      Fuzzer::cmdGrammar["yawGrammar"][mode.yawLogic][1]
      )(generator),
  };

  std::cout << 
    "X: " << joystickCommand.x << " ∈ [" <<
    Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][0] << ", " << 
    Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][1] << "]" << endl;

  std::cout << 
    "Y: " << joystickCommand.y << " ∈ [" << 
    Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][0] << ", " << 
    Fuzzer::cmdGrammar["horizontalGrammar"][mode.horizontalLogic][1] << "]" << endl;

  std::cout << 
    "Z: " << joystickCommand.z << " ∈ [" <<
    Fuzzer::cmdGrammar["verticalGrammar"][mode.verticalLogic][0] << ", " << 
    Fuzzer::cmdGrammar["verticalGrammar"][mode.verticalLogic][1] << "]" << endl;

  std::cout << 
    "Yaw: " << joystickCommand.yaw << " ∈ [" << 
    Fuzzer::cmdGrammar["yawGrammar"][mode.yawLogic][0] << ", " << 
    Fuzzer::cmdGrammar["yawGrammar"][mode.yawLogic][1] << "]" << endl;

  return joystickCommand;
}


// randomize command bit-level