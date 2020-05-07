/** @file dji_hms_impl.hpp
 *  @version 4.0
 *  @date APRIL 2020
 *
 *  @brief Battery API for DJI OSDK implement
 *  @Details Provide API to get battery's Dynamic data.
 *
 *  @Copyright (c) 2020 DJI
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

#ifndef ADVANCED_SENSING_DJI_BATTERY_IMPL_H
#define ADVANCED_SENSING_DJI_BATTERY_IMPL_H

#include "dji_type.hpp"
#include "dji_platform.hpp"
#include <vector>

namespace  DJI {
namespace OSDK{
#pragma pack(1)
/*!  The whole battery state*/
typedef struct DJIWholeBatteryStateImpl {
    uint32_t reserve :15;
    uint32_t isFakeSingleBatteryMode:1;
    uint32_t isSingleBatteryMode:1;
    uint32_t batteryPowerOffWarning:1;
    uint32_t batteryShutDownProtection:1; /*!Battery shutdown protection (throttle cannot be pushed in the air, forced landing)*/
    uint32_t powerLimit:1;                /*!Flight control imposes significant power limitations on flight*/
    uint32_t firstChargeNotFull:1;
    uint32_t batteryNotReady:1;           /*!The battery is not ready (the battery has not communicated just after being powered on,
                                           * or has not passed the first battery certification)*/
    uint32_t voltageNotSafety:1;          /*!Generally caused by low temperature, the battery has electricity,
                                           * but the battery voltage is too low*/
    uint32_t verySeriousLowVoltage:1;
    uint32_t batteryCommunicationError:1;
    uint32_t batteryCellError:1;
    uint32_t lv2LowVoltageAlarm:1;
    uint32_t lv1LowVoltageAlarm:1;
    uint32_t smartBatteryLanding:1;
    uint32_t smartBatteryGoHome:1;
    uint32_t seriousLowCapacityAlarm:1;
    uint32_t LowCapacityAlarm:1;
} DJIWholeBatteryStateImpl;

/*!  The whole battery information push,include:
 * 1.some raw battery data;
 * 2.calculated data;
 * 3.battery state
 */
typedef struct BatteryWholeInfoImpl {
    uint16_t remainFlyTime;
    uint16_t goHomeNeedTime;           /*! Time required for the gohome flight (s)*/
    uint16_t landNeedTime;             /*! Time required for the land flight (s).max value:100*/
    uint16_t goHomeNeedCapacity;       /*! Capacity required for the gohome flight (%).max value:100*/
    uint16_t landNeedCapacity;         /*! Capacity required for the land flight (%).max value:100*/
    float    safeFlyRadius;            /*! Safe flight area radius (m)*/
    float    capacityConsumeSpeed;     /*!(mAh/sec)*/
    DJIWholeBatteryStateImpl batteryState;
    uint8_t  goHomeCountDownState;     /*! Countdown status of smart battery gohome
                                        * 0/2:not in gohome state
                                        * 1  :in gohome state
                                        */
    uint8_t gohomeCountDownvalue;      /*! uint:s.max value:10*/
    uint16_t voltage;                  /*! mv*/
    uint8_t batteryCapacityPercentage; /*! uint:%.max value:100*/
    uint8_t lowBatteryAlarmThreshold:7;
    uint8_t lowBatteryAlarmEnable:1;
    uint8_t seriousLowBatteryAlarmThreshold:7;
    uint8_t seriousLowBatteryAlarmEnable:1;
    uint8_t batteryType:2;
    uint8_t reserve :6;
} BatteryWholeInfoImpl;

/*! Abnormal battery firmware status*/
typedef enum DJIBatteryFirmwareAbnormalImpl {
    DJI_APP_FIRAMWARE = 0,
    DJI_LOADER_FIRMWARE = 1,
} DJIBatteryFirmwareAbnormalImpl;


typedef struct DJISmartBatteryStateImpl {
    uint32_t firstLevelElectric       :1;
    uint32_t secondLevelElectric      :1;
    uint32_t firstLevelOverHeating    :1;
    uint32_t secondLevelOverHeating   :1;
    uint32_t firstLevelLowTemperature :1;
    uint32_t secondLevelLowTemperature:1;
    uint32_t shortCut :1;
    uint32_t cellUnderVoltage :5;    /*! 0:normal;other:Undervoltage core index(0x01-0x1F)*/
    uint32_t cellBreak :5;           /*! 0:normal;other:Undervoltage core index(0x01-0x1F)*/
    uint32_t selfCheckError :3;      /*! enum-type: DJI_SMART_BATTERY_SELF_CHECK*/
    uint32_t reset :1;
    uint32_t dischargeDuringStorage :1;
    uint32_t electricityOver  :1;
    uint32_t pressKeyClose    :1;
    uint32_t firmwareAbnormal :1;     /*! enum-type: DJI_BATTERY_FIRMWARE_ABNORMAL*/
    uint32_t sopWarning :1;
    uint32_t reserved2   :1;
    uint32_t batteryClosedReason :5;  /*! enum-type: DJI_BETTERY_CLOSED_REASON*/
    uint8_t  mosCONTROL :3;/*[0]CHG state；[1]DSG state；[2]ORING state*/
    uint8_t  isBatteryMosReady:1;
    uint8_t  batteryEndLife   :1;
    uint8_t  batPF  :1;
    uint8_t  batSOHState :2;                /*! enum-type: DJI_SMART_BATTERY_SOH_STATE*/
    uint8_t  maxCycleLimit:6;          /*! APP:cycle_limit*10*/
    uint8_t  reserved3    :2;
    uint16_t lessBattery  :1;
    uint16_t batteryCommunicationAbnormal:1;
    uint16_t voltageDiffOverRange1 :1; /*! can solve by Regrouping battery*/
    uint16_t voltageDiffOverRange2 :1; /*! can not solve by Regrouping battery*/
    uint16_t hasCellLowVoltages :1;
    uint16_t hasCellBreak :1;
    uint16_t hasDiffBatteryFirmware :1;
    uint16_t hasSocLow :1;
    uint16_t hasBatteryPowerSupplyAbnormal :1;
    uint16_t cellDiffVoltageOverRange:1;
    uint16_t isBatteryEmbed :1;
    uint16_t heatState :2;             /*!enum-type: DJI_SMART_BATTERY_HEAT_STATE*/
    uint16_t socState  :3;             /*!enum-type: DJI_SMART_BATTERY_SOC_WARNING*/
} DJISmartBatteryStateImpl;

typedef struct SmartBatteryDynamicInfoImpl {
    uint8_t  returnCode;              /*! 0:success*/
    uint8_t  batteryIndex;
    int32_t  currentVoltage;          /*! uint:mV*/
    int32_t  currentElectric;         /*!uint:mA*/
    uint32_t fullCapacity;            /*!uint:mAh*/
    uint32_t remainedCapacity;        /*!uint:mAh*/
    int16_t  batteryTemperature;      /*!uint:℃*/
    uint8_t  cellCount;
    uint8_t  batteryCapacityPercent;  /*!uint:%*/
    DJISmartBatteryStateImpl batteryState;
    uint8_t  batteryPrtocolVersion;   /*!Pool protocol version number. Note: [0x14, 0x3F] section (including boundary) version number is
                                       * reserved for agricultural machinery battery use*/
    uint8_t  batteryNumber;           /*! 1:no center board;
                                       *  n:battery number*/
    uint8_t  SOP;                     /*!Relative power percentage*/
} SmartBatteryDynamicInfoImpl;

const uint8_t MaxSmartBatteryNum = 2;

#pragma pack()
    class Vehicle;
    class DJIBatteryImpl {
    public:
        DJIBatteryImpl(Vehicle* vehicle);
        ~DJIBatteryImpl();

        Vehicle* vehicle;

    public:
        void setBatteryWholeInfo(const uint8_t *batteryData, const uint16_t dataLen);
        void getBatteryWholeInfo(BatteryWholeInfoImpl& batteryWholeInfo);

        bool createBatteryInfoLock();
        bool lockBatteryInfo();
        bool freeBatteryInfo();
        bool destroyBatteryInfoLock();

    private:
        BatteryWholeInfoImpl batteryWholeInfo;
        T_OsdkMutexHandle m_batteryLock;
    };
}
}


#endif //ADVANCED_SENSING_DJI_BATTERY_IMPL_H


