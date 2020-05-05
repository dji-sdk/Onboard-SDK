/** @file dji_hms.cpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief HMS(Health Management System) API for DJI OSDK
 *  @Details Provide API to subscribe Flight's Health State.
 *
 *  @Copyright (c) 2019 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <unistd.h>
#include <string>
#include "dji_vehicle.hpp"
#include "dji_linker.hpp"
#include "dji_hms.hpp"
#include "dji_hms_impl.hpp"
#include "dji_hms_internal.hpp"
#include "osdk_device_id.h"
#include "dji_status.hpp"
#include "osdk_command.h"
#include "dji_internal_command.hpp"

using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace DJI{
    namespace OSDK{
        extern HMSErrCodeInfo hmsErrCodeInfoTbl[dbHMSErrNum];
        extern bool replaceStr(string &str, const string oldReplaceStr, const string newReplaceStr);
    }
}

/*! @brief Compare HMS's pushing error code with the error code in the error code table,
* and print the prompt message.
*
*  @param djiHMSImpl pointer to djiHMSImpl
*  @param hmsPushData HMS's raw pushing data without a time stamp
*
*  @return bool pointer and status of subcribing flight check
*
*  @note Each error code will print different prompt information according to different states of the aircraft
*  (ground or air)
*/
static bool MarchErrCodeInfoTbl(DJIHMSImpl * djiHMSImpl, HMSPushData *hmsPushData);

/*! In hmsErrCodeInfoTbl's alarmInfo,we need to replace some identified str with real data.
 *  for example, %alarmid <-> 0x1a010040 , %index <-> 1, %component_index <-> 1*/
static void replaceHMSIdentifyStr(string &alarmInfo, uint32_t alarmId, uint8_t sensorIndex,
                                  uint8_t componentIndex);

static E_OsdkStat HMSRecvDataCallBack(struct _CommandHandle *cmdHandle,
                                      const T_CmdInfo *cmdInfo,
                                      const uint8_t *cmdData, void *userData);

DJIHMS::DJIHMS(Vehicle *vehicle):vehicle(vehicle)
{
    this->djiHMSImpl = new DJIHMSImpl(vehicle);
}

DJIHMS::~DJIHMS()
{
    if(djiHMSImpl) delete(djiHMSImpl);
}

bool
DJIHMS::subscribeHMSInf(bool enable, uint32_t timeOutMs) {
    uint8_t ackData;
    uint8_t cmd;
    T_CmdInfo cmdInfo = {0};
    T_CmdInfo ackInfo = {0};
    cmdInfo.cmdSet = V1ProtocolCMD::HMS::hmsStatus[0];
    cmdInfo.cmdId = V1ProtocolCMD::HMS::hmsStatus[1];
    cmdInfo.protoType = PROTOCOL_V1;
    cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
    cmdInfo.sender = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_PC, 6);
    cmdInfo.receiver = OSDK_COMMAND_HMSSERVICE_ID;
    cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
    cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
    cmdInfo.dataLen = sizeof(cmd);

    ackInfo.cmdSet = V1ProtocolCMD::HMS::hmsStatus[0];;
    ackInfo.cmdId = V1ProtocolCMD::HMS::hmsStatus[1];

    if(enable)
    {
        cmd = OSDKSubALLData;
    }
    else
    {
        cmd = ResetALLSubcriberExceptAPP;
    }
    vehicle->linker->sendSync(&cmdInfo, &cmd, &ackInfo, &ackData, timeOutMs, 2);
    if (ackData == OSDKSubSuccess) {
        if (enable)
        {
            this->enableListeningHmsData(true);
            DSTATUS("Subscribe all flight data success!");
        }
        else
        {
            this->enableListeningHmsData(false);
            DSTATUS("Unsubscribe all flight data success!");
        }
        return true;
    } else {
        DSTATUS("Subscribe/Unsubscribe all flight data failed!");
        return false;
    }
}

std::string DJIHMS::getHMSVersion()
{
    return djiHMSImpl->getHMSVersion();
}

HMSPushPacket DJIHMS::getHMSPushPacket()
{
    djiHMSImpl->lockHMSInfo();
    HMSPushPacket hmsPushPacket = djiHMSImpl->getHMSPushPacket();
    djiHMSImpl->freeHMSInfo();
    return hmsPushPacket;
}

uint8_t DJIHMS::getDeviceIndex()
{
    djiHMSImpl->lockHMSInfo();
    uint8_t data = djiHMSImpl->getDeviceIndex();
    djiHMSImpl->freeHMSInfo();
    return data;
}

bool DJIHMS::enableListeningHmsData(bool enable) {
    static T_RecvCmdHandle recvCmdHandle;
    static T_RecvCmdItem recvCmdItem;
    recvCmdItem.device = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_PC, 6);
    recvCmdItem.host = OSDK_COMMAND_HMSSERVICE_ID;
    recvCmdItem.cmdSet = V1ProtocolCMD::HMS::hmsPushData[0];
    recvCmdItem.cmdId = V1ProtocolCMD::HMS::hmsPushData[1];
    recvCmdItem.mask = MASK_HOST_XXXXXX_SET_ID;
    recvCmdItem.userData = this->djiHMSImpl;
    recvCmdHandle.protoType = PROTOCOL_V1;
    recvCmdHandle.cmdList = &recvCmdItem;
    recvCmdHandle.cmdCount = 1;

    if (enable)
    {
        recvCmdItem.pFunc = HMSRecvDataCallBack;
    }
    else
    {
        recvCmdItem.pFunc = nullptr;
    }
    return vehicle->linker->registerCmdHandler(&(recvCmdHandle));
}

static E_OsdkStat HMSRecvDataCallBack(struct _CommandHandle *cmdHandle,
                                      const T_CmdInfo *cmdInfo,
                                      const uint8_t *cmdData, void *userData) {
    if(!cmdData)
    {
        DERROR("null Data!");
        return OSDK_STAT_ERR;
    }
    DJIHMSImpl *djiHMSImpl = (DJIHMSImpl *)userData;

    djiHMSImpl->lockHMSInfo();
    djiHMSImpl->setDeviceIndex(cmdInfo->sender);
    djiHMSImpl->setHMSPushData(cmdData, cmdInfo->dataLen);
    djiHMSImpl->setHMSTimeStamp();
    djiHMSImpl->freeHMSInfo();
    MarchErrCodeInfoTbl(djiHMSImpl, &(djiHMSImpl->getHMSPushPacket().hmsPushData));

    return OSDK_STAT_OK;
}

static bool MarchErrCodeInfoTbl(DJIHMSImpl *djiHMSImpl, HMSPushData *hmsPushData) {
    if (!hmsPushData)
    {
        DSTATUS("HMS Push Data is nullptr!");
        return false;
    }
    OsdkOsal_TaskSleepMs(200);

    for (int i = 0; i < hmsPushData->errList.size(); i++)
    {
        for (int j = 0; j < sizeof(DJI::OSDK::hmsErrCodeInfoTbl) / sizeof(HMSErrCodeInfo); j++)
        {
            if (hmsPushData->errList[i].alarmID == DJI::OSDK::hmsErrCodeInfoTbl[j].alarmId)
            {
                switch (djiHMSImpl->vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>())
                {
                    case VehicleStatus::FlightStatus::IN_AIR:
                    {
                        if (DJI::OSDK::hmsErrCodeInfoTbl[j].flyAlarmInfo != "")
                        {
                            replaceHMSIdentifyStr(DJI::OSDK::hmsErrCodeInfoTbl[j].flyAlarmInfo,
                                                  hmsPushData->errList[i].alarmID,
                                                  hmsPushData->errList[i].sensorIndex,
                                                  djiHMSImpl->getDeviceIndex());
                            DSTATUS("TimeStamp: %ld.Info: %s\n", djiHMSImpl->getHMSPushPacket().timeStamp,
                                    DJI::OSDK::hmsErrCodeInfoTbl[j].flyAlarmInfo.c_str());
                        }
                        break;
                    }
                    default:
                    {
                        if (DJI::OSDK::hmsErrCodeInfoTbl[j].groundAlarmInfo != "")
                        {
                            replaceHMSIdentifyStr(DJI::OSDK::hmsErrCodeInfoTbl[j].groundAlarmInfo,
                                                  hmsPushData->errList[i].alarmID,
                                                  hmsPushData->errList[i].sensorIndex,
                                                  djiHMSImpl->getDeviceIndex());
                            DSTATUS("TimeStamp: %ld.Info:%s\n",djiHMSImpl->getHMSPushPacket().timeStamp,
                                    DJI::OSDK::hmsErrCodeInfoTbl[j].groundAlarmInfo.c_str());
                        }
                        break;
                    }
                }
            }
        }
    }

    return true;
}

static void replaceHMSIdentifyStr(string & alarmInfo, uint32_t alarmId, uint8_t sensorIndex,
                                  uint8_t componentIndex)
{
    const string oldReplaceAlarmIdStr        = "%alarmid";
    const string oldReplaceIndexStr          = "%index";
    const string oldReplaceComponentIndexStr = "%component_index";

    string newReplaceAlarmIdStr;
    string newReplaceIndexStr ;
    string newReplaceComponentIndexStr;

    sprintf((char *)newReplaceAlarmIdStr.data(), "0x%08X" , alarmId);
    sprintf((char *)newReplaceIndexStr.data(), "%d" , sensorIndex);
    sprintf((char *)newReplaceComponentIndexStr.data(), "%d" ,componentIndex);

    DJI::OSDK::replaceStr(alarmInfo, oldReplaceAlarmIdStr,newReplaceAlarmIdStr.data());
    DJI::OSDK::replaceStr(alarmInfo, oldReplaceIndexStr,newReplaceIndexStr.data());
    DJI::OSDK::replaceStr(alarmInfo, oldReplaceComponentIndexStr, newReplaceComponentIndexStr.data());
}