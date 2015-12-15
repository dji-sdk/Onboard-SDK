#ifndef DJI_MISSION_H
#define DJI_MISSION_H

#include "DJI_Config.h"

namespace DJI
{
namespace onboardSDK
{
#pragma pack(1)

typedef struct HotPointACKData
{
    uint8_t status;
    uint16_t radius; // in cm
    uint8_t failReasion;
    uint8_t palstance;
} HotPointADKData;

#pragma pack()

typedef struct MissionACKMap
{
    uint8_t code;
    const char* meaning;
} MissionACKMap;

} // onboardSDK
} // DJI

#endif // DJI_MISSION_H
