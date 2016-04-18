/*! @brief
 *  @file DJI_Codec.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Mission framework for DJI onboardSDK library
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Dec 16, 2015
 *  -* @author william.wu
 *
 * */

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
    uint8_t palstance;
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

void missionCallback(CoreAPI *This, Header *header, UserData userdata = 0);

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_MISSION_H
