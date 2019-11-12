/*! @file payloads/main_async.cpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief
 *  main for CameraManager usage in a Linux environment.
 *  Shows example usage of CameraManager asynchronous APIs.
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

#include "main_test.hpp"

using namespace DJI::OSDK;

TimeStamp           timeStamp   ;
SyncStamp           syncStamp   ;
Quaternion          q           ;
Vector3f            a           ;
Vector3f            v           ;
Vector3f            w           ;
VelocityInfo        vi          ;
GlobalPosition      gp          ;
RelativePosition    rp          ;
GPSInfo             gps         ;
RTK                 rtk         ;
Mag                 mag         ;
RC                  rc          ;
Gimbal              gimbal      ;
Status              status      ;
Battery             battery     ;
SDKInfo             info        ;

void unpackOne(uint16_t passFlag, FLAG flag, void* data, uint8_t*& buf, size_t size) {
  if (flag & passFlag)
  {
    memcpy((uint8_t*)data, (uint8_t*)buf, size);
    buf += size;
  }
}

E_OsdkStat unpackData(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo, 
                      const uint8_t *cmdData, void *userData) {
  uint8_t* pdata = (uint8_t *)cmdData;
  uint16_t passFlag;
  passFlag = *(uint16_t*)pdata;
  pdata += sizeof(uint16_t);

  printf("unpack broadcast data!\n");

  unpackOne(passFlag, FLAG_TIME        ,&timeStamp ,pdata,sizeof(timeStamp ));
  unpackOne(passFlag, FLAG_TIME        ,&syncStamp ,pdata,sizeof(syncStamp ));
  unpackOne(passFlag, FLAG_QUATERNION  ,&q         ,pdata,sizeof(q         ));
  unpackOne(passFlag, FLAG_ACCELERATION,&a         ,pdata,sizeof(a         ));
  unpackOne(passFlag, FLAG_VELOCITY    ,&v         ,pdata,sizeof(v         ));
  unpackOne(passFlag, FLAG_VELOCITY    ,&vi        ,pdata,sizeof(vi        ));
  unpackOne(passFlag, FLAG_ANGULAR_RATE,&w         ,pdata,sizeof(w         ));
  unpackOne(passFlag, FLAG_POSITION    ,&gp        ,pdata,sizeof(gp        ));
  unpackOne(passFlag, FLAG_POSITION    ,&rp        ,pdata,sizeof(rp        ));
  unpackOne(passFlag, FLAG_GPSINFO     ,&gps       ,pdata,sizeof(gps       ));
  unpackOne(passFlag, FLAG_RTKINFO     ,&rtk       ,pdata,sizeof(rtk       ));
  unpackOne(passFlag, FLAG_MAG         ,&mag       ,pdata,sizeof(mag       ));
  unpackOne(passFlag, FLAG_RC          ,&rc        ,pdata,sizeof(rc        ));
  unpackOne(passFlag, FLAG_GIMBAL      ,&gimbal    ,pdata,sizeof(gimbal    ));
  unpackOne(passFlag, FLAG_STATUS      ,&status    ,pdata,sizeof(status    ));
  unpackOne(passFlag, FLAG_BATTERY     ,&battery   ,pdata,sizeof(battery   ));
  unpackOne(passFlag, FLAG_DEVICE      ,&info      ,pdata,sizeof(info      ));
  return OSDK_STAT_OK;
}

T_RecvCmdItem s_broadcastCmdList[] = {
    PROT_CMD_ITEM(0, 0, 0x02, 0x00, MASK_HOST_DEVICE_SET_ID,
                  NULL, unpackData),
};

int main(int argc, char **argv) {
  T_RecvCmdHandle recvCmdHandle;

  recvCmdHandle.cmdList = s_broadcastCmdList;
  recvCmdHandle.cmdCount = sizeof(s_broadcastCmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle.protoType = PROTOCOL_SDK;

  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting. \n";
    return -1;
  }
  if(vehicle->linker->registerCmdHandler(&recvCmdHandle) != true) {
    std::cout << "register Cmd Handler failed, exiting. \n";
    return -1;
  }

  sleep(2);//wait for cmd handler working

  std::cout << "program exiting. \n";
  return 0;
}
