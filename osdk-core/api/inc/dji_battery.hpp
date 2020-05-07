/** @file dji_hms.cpp
 *  @version 4.0
 *  @date APRIL 2020
 *
 *  @brief Battery API for DJI OSDK
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

#ifndef ADVANCED_SENSING_DJI_BATTERY_H
#define ADVANCED_SENSING_DJI_BATTERY_H

#include "dji_type.hpp"
namespace DJI
{
namespace OSDK
{

#pragma pack(1)
/*!  The whole battery state*/
typedef struct DJIWholeBatteryState {
    uint32_t reserve1 :15;
    uint32_t isFakeSingleBatteryMode:1;
    uint32_t isSingleBatteryMode:1;
    uint32_t reserve2:4;
    uint32_t batteryNotReady:1;  /*!The battery is not ready (the battery has not communicated just after being powered on,
                                  * or has not passed the first battery certification)*/
    uint32_t voltageNotSafety:1; /*!Generally caused by low temperature, the battery has electricity,
                                  * but the battery voltage is too low*/
    uint32_t reserve3:3;
    uint32_t veryLowVoltageAlarm:1;
    uint32_t LowVoltageAlarm:1;
    uint32_t reserve4:2;
    uint32_t seriousLowCapacityAlarm:1;
    uint32_t LowCapacityAlarm:1;
} DJIWholeBatteryState;

/*!  The whole battery information push,include:
 * 1.some raw battery data;
 * 2.calculated data;
 * 3.battery state
 */
typedef struct BatteryWholeInfo {
    uint16_t remainFlyTime;
    uint16_t goHomeNeedTime;            /*! Time required for the gohome flight (s)*/
    uint16_t landNeedTime;              /*! Time required for the land flight (s).max value:100*/
    uint16_t goHomeNeedCapacity;        /*! Capacity required for the gohome flight (%).max value:100*/
    uint16_t landNeedCapacity;          /*! Capacity required for the land flight (%).max value:100*/
    float    safeFlyRadius;             /*! Safe flight area radius (m)*/
    float    capacityConsumeSpeed;      /*!(mAh/sec)*/
    DJIWholeBatteryState batteryState;
    uint8_t  goHomeCountDownState;      /*! Countdown status of smart battery gohome
                                         * 0/2:not in gohome state
                                         * 1  :in gohome state
                                         */
    uint8_t  gohomeCountDownvalue;      /*! uint:s.max value:10*/
    uint16_t voltage;                   /*! mv*/
    uint8_t  batteryCapacityPercentage; /*! uint:%.max value:100*/
    uint8_t  lowBatteryAlarmThreshold:7;
    uint8_t  lowBatteryAlarmEnable   :1;
    uint8_t  seriousLowBatteryAlarmThreshold:7;
    uint8_t  seriousLowBatteryAlarmEnable   :1;
    uint8_t  reserve;
} BatteryWholeInfo;


/*! Smart battery self-checkout error flag */
typedef enum DJISmartBatterySelfCheck {
    DJI_BATTERY_SELF_CHECK_NORAML        = 0,
    DJI_BATTERY_SELF_CHECK_NTC_ABNORAML  = 1,
    DJI_BATTERY_SELF_CHECK_MOS_ABNROMAL  = 2,
    DJI_BATTERY_SELF_CHECK_R_ABNORMAL    = 3,
    DJI_BATTERY_SELF_CHECK_CELL_DAMAGE   = 4,
    DJI_BATTERY_SELF_CHECK_CAL_EXP       = 5,
    DJI_BATTERY_SELF_CHECK_GAUGEPARM_EXP = 6,
    DJI_BATTERY_SELF_CHECK_RES           = 7,
} DJISmartBatterySelfCheck;

/*! Smart battery closed reason*/
typedef enum DJIBatteryClosedReasonImpl {
    DJI_BETTERY_CLOSED_NORMAL = 0,
    DJI_BETTERY_CLOSED_FORCED = 1,
    DJI_BETTERY_CLOSED_SCD    = 2,    /*!Discharge short circuit shutdown */
    DJI_BETTERY_CLOSED_OLD    = 3,    /*!Discharge overload shutdown*/
    DJI_BETTERY_CLOSED_OCD    = 4,    /*!Discharge overcurrent shutdown*/
    DJI_BETTERY_CLOSED_UVD    = 5,    /*!Discharge undervoltage shutdown*/
    DJI_BETTERY_CLOSED_OTD    = 6,    /*!Discharge over temperature shutdown*/
    DJI_BETTERY_CLOSED_SCC    = 16,   /*!Charging short circuit shutdown*/
    DJI_BETTERY_CLOSED_OCC    = 17,   /*!Charge overcurrent shutdown*/
    DJI_BETTERY_CLOSED_OVC    = 18,   /*!Charge overvoltage shutdown*/
    DJI_BETTERY_CLOSED_OVCER  = 19,   /*!Charger overvoltage shutdown*/
    DJI_BETTERY_CLOSED_LTC    = 20,   /*!Charging low temperature shutdown*/
    DJI_BETTERY_CLOSED_HTC    = 21,   /*!Charging high temperature shutdown*/
} DJIBatteryClosedReasonImpl;

/*!Smart battery cycle count status*/
typedef enum {
    DJI_SOH_NORMAL  = 0,
    DJI_SOH_ALERT   = 1,
    DJI_SOH_SAFE    = 2,
    DJI_SOH_RESERVE = 3,
} DJISmartBatterySohState;

/*! Smart battery heating status */
typedef enum DJISmartBatteryHeatState {
    DJI_NO_HEAT   = 0,
    DJI_IN_HEAT   = 1,
    DJI_KEEP_WARM = 2,
} DJISmartBatteryHeatState;

/*! Smart battery abnormal warning */
typedef enum DJISmartBatterySocWarning {
    DJI_SOC_NORMAL        = 0,
    DJI_SOC_ABNORMAL_HIGH = 1,
    DJI_SOC_JUMP_DOWN     = 2,
    DJI_SOC_JUMP_UP       = 3,
    DJI_SOC_INVALID       = 4,
    DJI_reserved1         = 5,
    DJI_reserved2         = 6,
    DJI_reserved3         = 7,
} DJISmartBatterySocWarning;

typedef struct DJISmartBatteryState {
    uint32_t reserved  :12;
    uint32_t cellBreak :5;            /*! 0:normal;other:Undervoltage core index(0x01-0x1F)*/
    uint32_t selfCheckError :3;       /*! enum-type: DJISmartBatterySelfCheck*/
    uint32_t reserved1 :7;
    uint32_t batteryClosedReason :5;   /*! enum-type: DJI_BETTERY_CLOSED_REASON*/
    uint8_t  reserved2 :6;/*[0]CHG state；[1]DSG state；[2]ORING state*/
    uint8_t  batSOHState :2;                /*! enum-type: DJISmartBatterySohState*/
    uint8_t  maxCycleLimit:6;          /*! APP:cycle_limit*10*/
    uint8_t  reserved3    :2;
    uint16_t lessBattery  :1;
    uint16_t batteryCommunicationAbnormal:1;
    uint16_t reserved4 :3;
    uint16_t hasCellBreak :1;
    uint16_t reserved5 :4;
    uint16_t isBatteryEmbed :1;        /*! 0:embed;1:unmebed*/
    uint16_t heatState :2;             /*!enum-type: DJISmartBatteryHeatState*/
    uint16_t socState  :3;             /*!enum-type: DJISmartBatterySocWarning*/
} DJISmartBatteryState;

typedef struct SmartBatteryDynamicInfo {
    uint8_t  reserve;
    uint8_t  batteryIndex;
    int32_t  currentVoltage;          /*! uint:mV*/
    int32_t  currentElectric;         /*!uint:mA*/
    uint32_t fullCapacity;            /*!uint:mAh*/
    uint32_t remainedCapacity;        /*!uint:mAh*/
    int16_t  batteryTemperature;      /*!uint:℃*/
    uint8_t  cellCount;
    uint8_t  batteryCapacityPercent;  /*!uint:%*/
    DJISmartBatteryState batteryState;
    uint8_t  reserve1;
    uint8_t  reserve2;
    uint8_t  SOP;                     /*!Relative power percentage*/
} SmartBatteryDynamicInfo;

#pragma pack()

class Vehicle;
class DJIBatteryImpl;

class DJIBattery
{
public:
    enum class RequestSmartBatteryIndex
    {
        FIRST_SMART_BATTERY  = 1,
        SECOND_SMART_BATTERY = 2,
    };

public:
    DJIBattery(Vehicle *vehicle = 0);
    ~DJIBattery();

    Vehicle* vehicle;
    DJIBatteryImpl* djiBatteryImpl;

public:
  /*! @remark Blocks until ACK frame arrives or timeout occurs
   *
   *  @brief Send subscribe request to your flight controller
   *         to get whole battery's information, blocking calls
   *
   *  @param enable whether subscribe battery Info
   *     true:subscribe
   *    false:unsubscribe
   *
   *  @return bool subscribe's ack result
   *     true:subscribe/unsubscribe success
   *    false:subscribe/unsubscribe failed
   */
    bool subscribeBatteryWholeInfo(bool enable);

  /*! @brief The interface of getting whole battery information
   *
   * @param batteryWholeInfo the whole information of battery
   * @return bool true:success;false:fail
   */
    bool getBatteryWholeInfo(BatteryWholeInfo& batteryWholeInfo);

  /*! @brief The interface of getting single battery's dynamic information
   *
   *  @param batteryIndex the index of battery.enum-type:RequestSmartBatteryIndex
   *  @param batteryDynamicInfo
   *  @return bool true:success;false:fail
   *
   *  @note If you input a invalid index,it will print "the battery index is overrange.Please recheck!";
   *        If get data failed,it will print "Get %d battery dynamic data fail!Please check the battery!"
   */
    bool getSingleBatteryDynamicInfo(const DJIBattery::RequestSmartBatteryIndex batteryIndex, SmartBatteryDynamicInfo& batteryDynamicInfo);
};

}
}
#endif //ADVANCED_SENSING_DJI_BATTERY_H
