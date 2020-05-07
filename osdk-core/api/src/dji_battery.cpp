/** @file dji_hms.cpp
 *  @version 4.0
 *  @date APRIL 2020
 *
 *  @brief Battery API for DJI OSDK
 *  @Details Provide API to get battery's Dynamic data.
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

#include "dji_battery.hpp"
#include "dji_linker.hpp"
#include "dji_internal_command.hpp"
#include "dji_battery_impl.hpp"
#include "dji_vehicle.hpp"
#include <stdio.h>

namespace DJI {
namespace OSDK {
#pragma pack(1)
typedef struct RequestBatteryDynamicInfoCmd
{
    uint8_t index;         /*! 0:Ttal dynamic data;
                            *  other:single battery dynamic dataerror code*/

    uint8_t requestType;   /*! 0:request data;
                            *  1:push data(CmdType should be 1,not use current)*/
    uint8_t stopPush;      /*! 0:start push
                            * 1:stop push
                            * (CmdType should be 1,not use current)*/
    uint8_t pushFrequency; /*! push frequency(uint:0.1HZ,range:0.1-25HZ)
                            * (CmdType should be 1,not use current)*/
} RequestBatteryDynamicInfoCmd;
#pragma pack()
}
}
DJIBattery::DJIBattery(Vehicle* vehicle)
:vehicle(vehicle)
{
    this->djiBatteryImpl = new DJIBatteryImpl(vehicle);
}

DJIBattery::~DJIBattery()
{
    if(djiBatteryImpl) delete(djiBatteryImpl);
}

static E_OsdkStat batteryRecvDataCallBack(struct _CommandHandle *cmdHandle,
                                      const T_CmdInfo *cmdInfo,
                                      const uint8_t *cmdData, void *userData)
{
    if(!cmdData)
    {
        DERROR("null Data!");
        return OSDK_STAT_ERR;
    }

    DJIBatteryImpl* djiBatteryImpl = reinterpret_cast<DJIBatteryImpl*>(userData);
    djiBatteryImpl->lockBatteryInfo();
    djiBatteryImpl->setBatteryWholeInfo(cmdData, sizeof(BatteryWholeInfo));
    djiBatteryImpl->freeBatteryInfo();

    return OSDK_STAT_OK;
}


bool DJIBattery::subscribeBatteryWholeInfo(bool enable)
{
    static T_RecvCmdHandle recvCmdHandle;
    static T_RecvCmdItem recvCmdItem;
    recvCmdItem.device = OSDK_COMMAND_FC_2_DEVICE_ID;
    recvCmdItem.cmdSet = V1ProtocolCMD::fc::batteryInfo[0];
    recvCmdItem.cmdId = V1ProtocolCMD::fc::batteryInfo[1];
    recvCmdItem.mask = MASK_HOST_XXXXXX_SET_ID;
    recvCmdItem.userData = this->djiBatteryImpl;
    recvCmdHandle.protoType = PROTOCOL_SDK;
    recvCmdHandle.cmdList = &recvCmdItem;
    recvCmdHandle.cmdCount = 1;
    if (enable)
    {
        recvCmdItem.pFunc = batteryRecvDataCallBack;
    }
    else
    {
        recvCmdItem.pFunc = nullptr;
    }
    return vehicle->linker->registerCmdHandler(&(recvCmdHandle));

}

bool DJIBattery::getBatteryWholeInfo(BatteryWholeInfo& batteryWholeInfo)
{
    djiBatteryImpl->lockBatteryInfo();
    BatteryWholeInfoImpl batteryWholeInfoImpl;
    djiBatteryImpl->getBatteryWholeInfo(batteryWholeInfoImpl);
    memcpy(&batteryWholeInfo, &batteryWholeInfoImpl, sizeof(BatteryWholeInfo));
    djiBatteryImpl->freeBatteryInfo();

    return true;
}

bool DJIBattery::getSingleBatteryDynamicInfo(const DJIBattery::RequestSmartBatteryIndex batteryIndex, SmartBatteryDynamicInfo& batteryDynamicInfo)
{
    uint8_t ackData[sizeof(SmartBatteryDynamicInfo)];
    uint8_t cmdData[sizeof(RequestBatteryDynamicInfoCmd)];
    RequestBatteryDynamicInfoCmd cmd;
    T_CmdInfo cmdInfo = {0};
    T_CmdInfo ackInfo = {0};
    cmdInfo.cmdSet = V1ProtocolCMD::BatteryCmd::getBatteryDynamicInfo[0];
    cmdInfo.cmdId = V1ProtocolCMD::BatteryCmd::getBatteryDynamicInfo[1];
    cmdInfo.protoType = PROTOCOL_V1;
    cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
    /*! sender:may be replace in the future */
    cmdInfo.sender = OSDK_COMMAND_PC_DEVICE_ID ;

    bool batteryIndexResult = true;
    switch (batteryIndex)
    {
      case DJIBattery::RequestSmartBatteryIndex::FIRST_SMART_BATTERY:
      {
        cmdInfo.receiver = OSDK_COMMAND_BATTERY_FIRST_DEVICE_ID;
        break;
      }
      case DJIBattery::RequestSmartBatteryIndex::SECOND_SMART_BATTERY:
      {
        cmdInfo.receiver = OSDK_COMMAND_BATTERY_SECOND_DEVICE_ID;
        break;
      }

      default:
      {
        batteryIndexResult = false;
        break;
      }
    }

    if (!batteryIndexResult)
    {
      DSTATUS("the battery index is overrange.Please recheck!\n");
      return false;
    }

    cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_RECEIVE_ACK ;
    cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
    cmdInfo.dataLen = sizeof(cmd);

    ackInfo.cmdSet = V1ProtocolCMD::BatteryCmd::getBatteryDynamicInfo[0];
    ackInfo.cmdId = V1ProtocolCMD::BatteryCmd::getBatteryDynamicInfo[1];

    cmd.index = static_cast<uint8_t >(batteryIndex);
    /*! request data */
    cmd.requestType   = 0;
    cmd.stopPush      = 1;
    /*! 1HZ (not use current)*/
    cmd.pushFrequency = 1;
    memcpy(cmdData, &cmd, sizeof(cmd));
    E_OsdkStat ret = vehicle->linker->sendSync(&cmdInfo, cmdData, &ackInfo, ackData, 500, 2);

    if (ret != OSDK_STAT_OK)
    {
      DSTATUS("Get %d battery dynamic data fail!Please check the battery!\n", cmd.index);
      return false;
    }
    memcpy(&batteryDynamicInfo, ackData, sizeof(batteryDynamicInfo));

    return true;
}