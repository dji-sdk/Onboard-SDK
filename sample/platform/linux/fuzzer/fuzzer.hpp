#pragma once

#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include "dji_flight_controller.hpp"
#include "dji_flight_joystick_module.hpp" // for enum values

using namespace std;
using namespace DJI::OSDK;

class Fuzzer
{
    private:
        map<string, vector<int>> modeGrammar;

        // map<string, map<int, vector<float>*>*> cmdGrammar; //the vector values somehow go missing if pointers are used
        map<string, map<int, vector<float>>> cmdGrammar; 

    public:
        Fuzzer();

        // randomize mode grammer-based
        void initializeModeGrammar();
        FlightController::JoystickMode generateModeWithGrammar();
        
        // randomize command grammar-based
        void initializeCommandGrammar();
        FlightController::JoystickCommand generateCommandWithGrammar(FlightController::JoystickMode mode);
        
        // randomize joystick action bit-level
};