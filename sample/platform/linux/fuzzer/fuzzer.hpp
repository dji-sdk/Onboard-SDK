#pragma once

#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include "dji_flight_controller.hpp"

using namespace std;

class Fuzzer
{
    private:
        map<string, vector<int>> modeGrammar;

    public:
        Fuzzer();
        void initializeModeGrammar();
        DJI::OSDK::FlightController::JoystickMode generateModeWithGrammar();
};

// randomize mode bit-level


// randomize command values


// randomize command bit-level

