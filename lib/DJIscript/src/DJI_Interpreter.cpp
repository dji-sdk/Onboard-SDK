#include "DJI_Interpreter.h"

using namespace DJI::onboardSDK;

Interpreter::Interpreter(CoreAPI *controlAPI)
{
    script = new Script(controlAPI);
}

