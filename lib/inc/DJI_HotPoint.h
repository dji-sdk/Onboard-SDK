/** @file DJI_Hotpoint.h
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  HotPoint API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All right resserved.
 *
 */

#ifndef DJI_HOTPOINT_H
#define DJI_HOTPOINT_H

#include "DJI_Mission.h"

namespace DJI
{
namespace onboardSDK
{

class HotPoint
{
  public:
#pragma pack(1)
  typedef struct YawRate
  {
    uint8_t clockwise;
    float32_t yawRate;
  } YawRate;
#pragma pack()

  enum View
  {
    VIEW_NORTH = 0,
    VIEW_SOUTH = 1,
    VIEW_WEST = 2,
    VIEW_EAST = 3,
    VIEW_NEARBY = 4
  };

  enum YawMode
  {
    YAW_AUTO = 0,
    YAW_INSIDE = 1,
    YAW_OUTSIDE = 2,
    YAW_CUSTOM = 3,
    YAW_STATIC = 4,
  };

  public:
  HotPoint(CoreAPI *ControlAPI = 0);
  void initData();

  /*! @note API functions
   *  @attention difference between set and update
   *  Set functions only change the HotPoint data in this class,
   *  Update functions will change the Mission status.
   *  In other words: drone will response update functions immediately.
   * */

  void start(CallBack callback = 0, UserData userData = 0);
  HotPointStartACK start(int timer);
  void stop(CallBack callback = 0, UserData userData = 0);
  MissionACK stop(int timer);
  void pause(bool isPause, CallBack callback = 0, UserData userData = 0);
  MissionACK pause(bool isPause, int timer);

  void updateYawRate(YawRate &Data, CallBack callback = 0, UserData userData = 0);
  MissionACK updateYawRate(YawRate &Data, int timer);
  void updateYawRate(float32_t yawRate, bool isClockwise, CallBack callback = 0,
      UserData userData = 0);
  void updateRadius(float32_t meter, CallBack callback = 0, UserData userData = 0);
  MissionACK updateRadius(float32_t meter, int timer);
  void resetYaw(CallBack callback = 0, UserData userData = 0);
  MissionACK resetYaw(int timer);

  void readData(CallBack callback = 0, UserData userData = 0);
  MissionACK readData(int timer);

  public:
  //! @note data access functions
  void setData(const HotPointData &value);
  void setHotPoint(float64_t longtitude, float64_t latitude, float64_t altitude);
  void setHotPoint(GPSPositionData gps);
  void setRadius(float64_t meter);
  void setYawRate(float32_t defree);
  void setClockwise(bool isClockwise);
  void setCameraView(View view);
  void setYawMode(YawMode mode);

  HotPointData getData() const;

  public:
  static void startCallback(CoreAPI *api, Header *protocolHeader, UserData userdata = 0);
  static void readCallback(CoreAPI *api, Header *protoclHeader, UserData userdata);

  private:
  CoreAPI *api;
  HotPointData hotPointData;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_HOTPOINT_H
