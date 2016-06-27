#### 本文档已停止维护, 请移步到DJI开发者官网的 <https://developer.dji.com/onboard-sdk/documentation/introduction/virtual-rc-protocol.html> 查看最新版本. 

# 虚拟遥控

## 简介

虚拟遥控器是 Onboard SDK 中用来模拟遥控器数值进而控制飞机飞行的 API 接口。

开发者可以无需获取飞机控制权而直接通过虚拟遥控控制飞行。开发者需要持续不断的发送通道数据，如果在连续一秒内飞机未收到任何通道数据，飞机会退出虚拟遥控逻辑。


## 命令集与命令码

|API 等级|命令集|命令码|简介|
|---------|-------|------|-----------|
|0|0x05|0x00|获取和释放虚拟遥控控制权|
|0|0x05|0x01|虚拟遥控器通道值|


## 数据结构

```c
/* Struct of RC Enable/Disable */
typedef struct
{
    uint8_t on_off: 1; //open (1) or close (0) the virtual RC
    uint8_t if_switch_back_to_real_RC: 1; //if switch back to real RC (1) or run RC-lost logic directly (0)
    uint8_t reserved: 6; //reserved
}virtual_rc_enable;

/* Struct of RC Data */
typedef struct
{
    uint32_t rc_raw_data[16]; //developer designed channel value
}virtual_rc_data
```

## 注意

1. 若飞机在1s内未接收到任何 `virtual_rc_data` 数据，则会自动退出虚拟遥控逻辑。若`if_switch_back_to_real_RC` 被设置成 1，则在断开逻辑后切换至真实遥控器受控；若被设置成 0，则直接进入遥控器失联逻辑。

2. 各个通道的含义和具体的取值需要根据开发者的需求在调参软件中进行定义。

    以下我们提供一组默认物理通道的映射配置用用来模拟 DJI 原装遥控器：

    |功能通道|物理通道|通道类型|取值|
    |------|-------|-------|---|
    |Roll|0|摇杆|[1024-660, 1024+660]|
    |Pitch|1|摇杆|[1024-660, 1024+660]|
    |Throttle|2|摇杆|[1024-660, 1024+660]|
    |Yaw|3|摇杆|[1024-660, 1024+660]|
    |MODE|6|三档开关|1552(P),1024(A),496(F)|
    |GO_HOME|5|按钮|N/A|
    |GEAR|4|二档开关|1684(Up),1324(Down)|
    

    例如：
    
    * 解锁/锁定电机：
    
        ```
        data[0] = 1024-660; (Roll)
        data[1] = 1024-660; (Pitch)
        data[2] = 1024-660; (Throttle)
        data[3] = 1024+660; (Yaw)
        data[4] = 1324; (Gear)
        data[6] = 1552; (Mode)
        ```
        
    * 飞机上升
    
        ```
        data[0] = 1024; (Roll)
        data[1] = 1024; (Pitch)
        data[2] = 1024+660; (Throttle)
        data[3] = 1024; (Yaw)
        data[4] = 1324; (Gear)
        data[6] = 1552; (Mode)
        ```
