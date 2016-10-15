/** @file DJI_Follow.h
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Follow API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All right reserved.
 *
 */

#ifndef DJI_FOLLOW_H
#define DJI_FOLLOW_H

#include "DJI_Mission.h"

namespace DJI
{
namespace onboardSDK
{

#pragma pack(1)

typedef struct FollowTarget
{
  float64_t latitude;
  float64_t longitude;
  uint16_t height;
  uint16_t angle;
} FollowTarget;

typedef struct FollowData
{
  uint8_t mode;
  uint8_t yaw;
  FollowTarget target;
  uint8_t sensitivity;
} FollowData;

#pragma pack()

//! Follow class encapsulates all follow control related functions provided by the DJI OnboardSDK.
class Follow
{
  public:
  enum MODE
  {
    MODE_RELATIVE = 0,
    MODE_ROUTE = 1, //! @note Flight control not support now
    MODE_SMART = 2  //! @note Flight control not support now
  };

  enum YAW_TYPE
  {
    YAW_TOTARGET = 0,
    YAW_CUSTOM = 1
  };

  enum SENSITIVITY
  {
    SENSE_LOW = 0, //! @note Flight control not support now
    SENSE_MID = 1,
    SENSE_HIGH = 2 //! @note Flight control not support now
  };

  public:
  Follow(CoreAPI *ControlAPI = 0);
  void resetData();      
  void start(FollowData *Data = 0, CallBack callback = 0, UserData userData = 0);
  MissionACK start(FollowData *Data, int timeout);
  void stop(CallBack callback = 0, UserData userData = 0);
  MissionACK stop(int timer);
  //! @note true for pause, false for resume
  void pause(bool isPause, CallBack callback = 0, UserData userData = 0);
  MissionACK pause(bool isPause, int timer);
  void updateTarget(FollowTarget target); //! @note no ack command
  void updateTarget(float64_t latitude, float64_t longitude, uint16_t height,
      uint16_t angle); //! @note no ack command

  public:
  void setData(const FollowData &value);
  void setMode(const MODE mode);
  void setTarget(FollowTarget target);
  void setYawType(const YAW_TYPE type);
  void setSensitivity(const SENSITIVITY sense);

  FollowData getData() const;

  private:
  CoreAPI *api;
  FollowData followData;
};

} //! namespace onboardSDK
} //! namespace DJI

#endif //! DJI_FOLLOW_H
