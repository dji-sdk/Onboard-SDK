//
// Created by dji on 2020/4/2.
//

#ifndef ADVANCED_SENSING_DJI_BATTERY_H
#define ADVANCED_SENSING_DJI_BATTERY_H

#include "dji_type.hpp"
namespace DJI
{
namespace OSDK
{

#pragma pack(1)
    // ========== structs ==========
// # 飞控提供的电池状态
typedef struct DJIFcBatteryState {
// 保留
    uint32_t resv :15;
// 双电池的且允许单电池起飞的机型才会有这个标志位。
// 表示当前是单电池模式，但是有一块假电池在另一个位置。
    uint32_t fake_single_battery :1;
// 双电池的且允许单电池起飞的机型才会有这个标志位。
// 表示当前是单电池模式
    uint32_t single_battery :1;
// 飞机即将执行电池关机保护警告
    uint32_t battery_power_off_warning:1;
// 电池关机保护（空中无法推油门，强制落地）
    uint32_t battery_power_off :1;
// 飞控对飞行进行了明显的功率限制
    uint32_t power_limit :1;
// 第一次充电没有充满（电池推送过来的）
    uint32_t first_charge_not_full :1;
// 电池没有准备好（刚上电电池没通讯，或者还没通过第一次电池认证）
    uint32_t not_ready :1;
// 当前电池电压不允许起飞（一般是低温引起的，电池还有电，但是电池电压过低）
    uint32_t voltage_not_safety :1;
// 严重低电压
    uint32_t voltage_very_low :1;
// 电池通讯异常
    uint32_t communication_error :1;
// 电芯异常
    uint32_t cell_error :1;
// 二级低电压警报
    uint32_t voltage_lv2 :1;
// 一级低电压警报
    uint32_t voltage_lv1 :1;
// 智能电量下降
    uint32_t smart_battery_landing :1;
// 智能电量返航
    uint32_t smart_battery_gohome :1;
// 严重低电量报警
    uint32_t capacity_lv2 :1;
// 低电量报警
    uint32_t capacity_lv1 :1;
} DJIFcBatteryState;
// ========== structs end ======

// ========== commands =========
// # 电池相关的OSD信息推送
// 该命令用于推送飞控这边关于电池的信息，包括：
// 1. 部分电池原始信息
// 2. 飞控根据电池数据计算出来的飞行辅助信息
// 3. 飞控根据电池数据判断的电池状态
typedef struct BatteryWholeInfo {
    // 剩余飞行时间（s）
    uint16_t remain_fly_time;
// 返航过程需要的时间（s）
    uint16_t need_gohome_time;
// 下降过程需要的时间（s）
    uint16_t need_land_time; // max value: 100
// 返航需要的电量（%）
    uint16_t gohome_capacity; // max value: 100
// 下降需要的电量（%）
    uint16_t land_capacity;
// 安全飞行区域半径（m）
    float safe_fly_radius;
// 耗电速度(mAh/sec)
    float capacity_consume_speed;
// 飞控电池状态
    DJIFcBatteryState battery_state;
// 智能电量返航倒计时状态：
// * 0 = 没有返航
// * 1 = 正在返航
// 由于历史原因，该字段有可能为2，如果为2则表示没有返航，和0含义一致
    uint8_t gohome_cntdown_state; // max value: 1
// 智能电量返航倒计时（s），0表示没有倒计时
    uint8_t gohome_cntdown_value; // max value: 10
// 飞机动力电压（mv）
    uint16_t voltage;
// 电池电量百分比（%）
    uint8_t battery_capacity_percentage; // max value: 100
// 低电量报警阈值
    uint8_t capacity_protect_lv1_threshold:7;
// 是否使能低电量报警
    uint8_t capacity_protect_lv1_enable :1;
// 严重低电量报警阈值
    uint8_t capacity_protect_lv2_threshold:7;
// 是否使能严重低电量报警
    uint8_t capacity_protect_lv2_enable :1;
// 电池类型
    uint8_t battery_type:2;
// 保留
    uint8_t rsvd :6;
} BatteryWholeInfo;

typedef struct DJISmartBatteryState {
// 放电一级电流
    uint32_t first_level_current :1;
// 放电二级电流
    uint32_t second_level_current :1;
// 放电一级过温
    uint32_t first_level_overheating :1;
// 放电二级过温
    uint32_t second_level_overheating :1;
// 放电一级低温
    uint32_t first_level_low_temperature :1;
// 放电二级低温
    uint32_t second_level_low_temperature:1;
// 放电短路
    uint32_t short_cut :1;
// 电芯欠压 0=正常 其他=欠压电芯索引(0x01-0x1F)
    uint32_t under_voltage_cell :5;
// 电芯损坏 0=正常 其他=损坏电芯索引(0x01-0x1F)
    uint32_t break_cell :5;
// 电池自检异常
    uint32_t self_check :3; // enum-type: DJI_SMART_BATTERY_SELF_CHECK
// 看门狗异常复位
    uint32_t reset :1;
// 存储过程中放电
    uint32_t self_discharge :1;
// 电流溢出标识
    uint32_t electricity_over :1;
// 按键关机请求标识
    uint32_t press_key_close :1;
// 电池固件异常
    uint32_t firmware_abnormal :1; // enum-type: DJI_BATTERY_FIRMWARE_ABNORMAL
// 电池SOP不足告警（仅在允许起飞条件下提示，是否允许起飞仍依据各机型SOP策略判断）
    uint32_t sop_warning :1;
// 保留
    uint32_t reserved2 :1;
// 电池关断原因
    uint32_t closed_reason :5; // enum-type: DJI_BETTERY_CLOSED_REASON
// [0]CHG管状态；[1]DSG管状态；[2]ORING管状态
    uint8_t MOS_CONTROL :3;
// 电池MOS是否准备就绪
    uint8_t BATTERY_READ:1;
// 电池寿命终止
    uint8_t bat_endlife :1;
// 电池PF
    uint8_t bat_PF :1;
// 电池电芯状态
    uint8_t bat_SOH :2; // enum-type: DJI_SMART_BATTERY_SOH_STATE
// 电池最高限制循环数（APP实际显示cycle_limit*10）
    uint8_t cycle_limit:6;
// 保留
    uint8_t reserved3 :2;
// 插入电池太少
    uint16_t less_battery :1;
// 通信异常（具体哪块电池通信异常，需二级查询）
    uint16_t communication_exception:1;
// 压差过大，可以通过移动重新组合电池
    uint16_t diff_voltage_reaarrange:1;
// 压差过大，且不可通过移动重新组合电池
    uint16_t diff_voltage :1;
// 有电池电芯欠压
    uint16_t has_low_voltages :1;
// 有电池电芯损坏
    uint16_t has_break_cell :1;
// 有电池固件版本不一致
    uint16_t has_diff_firmware :1;
// 有电池电量小于10%
    uint16_t has_soc_low :1;
// 有电池供电异常
    uint16_t has_battery_exception :1;
// 电芯压差过大
    uint16_t cell_diff_voltage_large:1;
// 电池在位状态
    uint16_t present_state :1;
// 加热状态
    uint16_t heat_state :2; // enum-type: DJI_SMART_BATTERY_HEAT_STATE
// 电池SOC电量状态
    uint16_t soc_warning :3; // enum-type: DJI_SMART_BATTERY_SOC_WARNING
} DJISmartBatteryState;

typedef struct SmartBatteryDynamicInfo {
//    // 返回码（推送时无此字段，请求返回时有此字段）
//    uint8_t return_code;
// 动态数据
// 0=总动态数据 其他=单个电池动态数据
    uint8_t index;
// 当前电压,mV
    int32_t current_voltage;
// 当前电流,mA
    int32_t current_current;
// 电池充满容量,mAh
    uint32_t full_capacity;
// 电池剩余容量,mAh
    uint32_t remained_capacity;
// 温度,0.1℃
    int16_t temperature;
// 电芯数量
    uint8_t cell_count;
// 容量百分比(0-100)
    uint8_t capacity_percent;
// 电池当前状态
    DJISmartBatteryState bettery_state;
// 电池协议版本号。注意：[0x14, 0x3F] 区间（含边界）版本号预留给农机电池使用
    uint8_t prtocol_version;
// 1-没有中心板 n-有中心板时由中心板实际检测到的电池个数(index为0的时候)
    uint8_t center_board_info;
// 相对功率百分比
    uint8_t SOP;
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
    bool subscribeBatteryWholeInfo(bool enable);
    bool subscribeSingleBatteryDynamicInfo(bool enable, const DJIBattery::RequestSmartBatteryIndex batteryIndex);

    void getBatteryWholeInfo(BatteryWholeInfo& batteryWholeInfo);
    void getSingleBatteryDynamicInfo(const DJIBattery::RequestSmartBatteryIndex batteryIndex, SmartBatteryDynamicInfo& batteryDynamicInfo);

private:
    bool enableListeningSingleBatteryDynamicInfo(bool enable);
};

}
}
#endif //ADVANCED_SENSING_DJI_BATTERY_H
