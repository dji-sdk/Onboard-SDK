/** @file DJI_Mission.h
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Mission Framework for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All right reserved.
 *
 */

#ifndef DJI_MISSION_H
#define DJI_MISSION_H

#include "DJI_Config.h"
#include "DJI_API.h"

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
  uint8_t yawRate;
} HotPointADKData;

//! @todo unify the naming style
typedef struct GSPushData
{
  uint8_t type;
  uint8_t data_1;
  uint8_t data_2;
  uint8_t data_3;
  uint8_t data_4;
  uint8_t data_5;
} GSPushData;

#pragma pack()

typedef struct MissionACKMap
{
  uint8_t code;
  const char *meaning;
} MissionACKMap;

void missionCallback(CoreAPI *api, Header *protocolHeader, UserData userdata = 0);

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_MISSION_H
