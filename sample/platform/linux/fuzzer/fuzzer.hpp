#pragma once

#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include "dji_flight_controller.hpp"
// #include <random>
// #include "dji_flight_joystick_module.hpp" // for enum values

using namespace std;

class Fuzzer
{
    private:
        map<string, vector<int>> modeGrammar;
        map<int, vector<float>> cmdGrammar; 
        
    public:
        Fuzzer();

        // randomize mode grammer-based
        void initializeModeGrammar();
        DJI::OSDK::FlightController::JoystickMode generateModeWithGrammar();
        
        // randomize command grammar-based
        void initializeCommandGrammar();
        DJI::OSDK::FlightController::JoystickCommand generateCommandWithGrammar(DJI::OSDK::FlightController::JoystickMode mode);
        
        // randomize joystick action bit-level
};