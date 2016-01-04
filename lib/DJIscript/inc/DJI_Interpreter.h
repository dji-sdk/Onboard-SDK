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
