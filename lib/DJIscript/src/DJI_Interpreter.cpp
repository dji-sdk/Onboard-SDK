/*! @file DJI_Interpreter.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  DJI Interpreter for DJI Onboard SDK Command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "DJI_Interpreter.h"

using namespace DJI::onboardSDK;

Interpreter::Interpreter(CoreAPI *controlAPI)
{
  script = new Script(controlAPI);
}

