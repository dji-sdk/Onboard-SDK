/*! @brief
 *  @file DJI_Codec.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Virtual Radio Control API for DJI onboardSDK library
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

#ifndef DJI_VIRTUALRC_H
#define DJI_VIRTUALRC_H

#include "DJI_API.h"

namespace DJI
{
namespace onboardSDK
{

class VirtualRC
{
  public:
    //! @todo spelling correction
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
     *      reset code;
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
     *      myData = myAPIToSetupDataFromGroundStation();
     *   }
     *
     *   Thread 2:
     *   while(1)
     *   {
     *      sendData(myData);
     *      msleep(200);
     *   }
     *
     *   When your drone lose signal, it will keep the recent command sent by your API:
     *   myAPIToSetupDataFromGroundStation();
     *   Somehow, you will never get your drone back in one pice, if this tragedy happend.
     *
     *   @note API "sendData();" need to be called above 2Hz, and not greater than 25hz.
     *   @note API "sendSafeModeData();" will lead your drone hover;
     *
     * */
    void setControl(bool enable, CutOff cutoffType);
    void sendData(VirtualRCData Data);
    void sendData();
    void resetData();
    void sendSafeModeData();

    VirtualRCData getVRCData() const;
    RadioData getRCData() const;

    void setVRCData(const VirtualRCData &value);

    bool isVirtualRC() const;

  public:
    static RadioData toRadioData(VirtualRCData &vData);
    static VirtualRCData toVirtualRCData(RadioData &rData);

  public:
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
    VirtualRCData data;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_VIRTUALRC_H
