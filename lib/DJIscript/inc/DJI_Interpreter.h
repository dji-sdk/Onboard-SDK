/*! @file DJI_Interpreter.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  DJI Interpreter header for DJI Onboard SDK Command line example. Work in progress.
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef DJI_INTERPRETER_H
#define DJI_INTERPRETER_H

#include "DJI_Script.h"

namespace DJI
{
namespace onboardSDK
{


typedef struct ScriptName
{
  char* name;
  Task task;
}ScriptName;

//! The interpreter class is currently not used. Functionality will change in the future.
class Interpreter
{
public:
  Interpreter(CoreAPI *controlAPI);

private:
  Script *script;

public:
  ScriptName *list;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_INTERPRETER_H
