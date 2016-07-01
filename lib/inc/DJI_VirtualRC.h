/**@file DJI_VirtualRC.h
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Virtual Radio Control API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */


#ifndef DJI_VIRTUALRC_H
#define DJI_VIRTUALRC_H

#include "DJI_API.h"

namespace DJI
{
namespace onboardSDK
{

//! VirtualRC class has all the methods to mimic the RC functionality via OnboardSDK. 
class VirtualRC
{
  public:
  enum CutOff
  {
    CutOff_ToLogic = 0,
    CutOff_ToRealRC = 1
  };

  public:
  VirtualRC(CoreAPI *ControlAPI = 0);

  /*! @attention Safety notes
   * You must use these methods below really carefully with following rules:
   * - Use API setControl(bool enable, CutOff cutoffType); only once in your main loop;
   * - Check your control device status by useing the following method:
   *   if(getControlDevice() != CONTROL_BY_VIRTUALRC)
   *   {
   *    reset code;
   *   }
   * - @attention Your reset code must not be "setControl(true);" without any status check
   *   It may cause your drone became a nut drone.
   * - @note You could quit your VRC(Virtual Remot Control) by switching your remote
   *   controller out of modeF
   * - @attention Most dangerous, setControl(true); must not be called over 0.5Hz or your
   *   drone will be locked in a logic-checking loop. And you cannot exit VirtualRC mode by
   *   switching your remote controller's mode. Actually, in this situation your drone will
   *   be a full automatically controlled by it self. It will keep flying until its bettery
   *   is empty or your code make it stop.
   * - @attention If you do not know what reset code you need to write, please just output
   *   your datalog and keep it empty.
   * - @attention It would be realy dangous if you keep calling sendData(); in a loop with out
   *   control-losing protection. like :
   *
   *   Global:
   *   VirtualRCData myData;
   *
   *   Thread 1:
   *   while(1)
   *   {
   *    myData = myAPIToSetupDataFromGroundStation();
   *   }
   *
   *   Thread 2:
   *   while(1)
   *   {
   *    sendData(myData);
   *    msleep(200);
   *   }
   *
   *   When your drone lose signal, it will keep the recent command sent by your API:
   *   myAPIToSetupDataFromGroundStation();
   *   This may result in a catastrophic crash.
   *
   *   @note API "sendData();" need to be called above 2Hz, and not greater than 25hz.
   *   @note API "sendSafeModeData();" will lead your drone hover;
   *
   * */
  void setControl(bool enable, CutOff cutoffType);
  void sendData(VirtualRCData Data);
  
  void sendData();                
  void resetData();               
  
  void sendSafeModeData();        //! @deprecated This function will not be maintained and will be deprecated in a future release - please use neutralVRCSticks() instead. 
  void neutralVRCSticks();        //!New function - this will replace sendSafeModeData() in a future release. 

  VirtualRCData getVRCData() const;
  //! @warning return type will change to RCData in a future release 
  RadioData getRCData() const;

  void setVRCData(const VirtualRCData &value);

  bool isVirtualRC() const;

  public:
  //! @warning old interface. Will be replaced by toRCData (see below) in a future release.
  static RadioData toRadioData(VirtualRCData &vData);
  static RCData toRCData(VirtualRCData &vData);

  static VirtualRCData toVirtualRCData(RadioData &rData);

  public:
  CoreAPI *getApi() const;
  void setApi(CoreAPI *value);

  private:
  CoreAPI *api;
  VirtualRCData vrcData;
};

} //! namespace onboardSDK
} //! namespace DJI

#endif //! DJI_VIRTUALRC_H
