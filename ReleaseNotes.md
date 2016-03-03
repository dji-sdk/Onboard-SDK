[English Version](#summary-of-changes-from-firmware-23-to-firmware-31) | [中文版本](#固件与sdk接口更新从23升级至31)

#Summary of changes from firmware 2.3 to firmware 3.1

**Important**: Developers who update from 2.3 to 3.1 should update their programs (or the library) accordingly as well. This file is an index about what has been changed. For the detail usages and definations of new content, please refer to its corresponding document.  

Developers beginning from firmware 3.1 should ignore this file and check out the documentation directly.


## Protocol Updates

### New

|CMD SET|CMD ID|Description|
|-------|------|-----------|
|0x01|0x05|Arm/Disarm the Drone|
|0x03||Gound Station Protocol|
|0x04|0x00|Synchronize Timestamp|
|0x05||Virtual RC Protocol|


### Changed


|CMD SET|CMD ID|Difference|2.3|3.1|
|-------|------|---|---|---|
|0x00|0x00|Version Query Result|2.3.10.0|3.1.10.0|
|0x00|0x01|App Level Removed|`typedef struct{ `<br>&nbsp;&nbsp;`uint32_t app_id;`<br>&nbsp;&nbsp;`uint32_t app_level;`<br>&nbsp;&nbsp;`uint32_t app_version;`<br>&nbsp;&nbsp;`uint8_t app_bundle_id[32];`<br>`} sdk_activation_info_t;`|`typedef struct{ `<br>&nbsp;&nbsp;`uint32_t app_id;`<br>&nbsp;&nbsp;~~`uint32_t app_level;`~~<br>&nbsp;&nbsp;`uint32_t app_version;`<br>&nbsp;&nbsp;`uint8_t app_bundle_id[32];`<br>`} sdk_activation_info_t;`|
|0x00|0x01|Activation SDK Version|0x02030a00|0x03010a00|
|0x01|0x03|Attitude Control Flag|`bit7&6: HORI_MODE`<br>`bit5&4: VERT_MODE`<br>`bit3: YAW_MODE`<br>`bit2&1: HORI_FRAME`<br>`bit0: YAW_FRAME`|`bit7&6: HORI_MODE`<br>`bit5&4: VERT_MODE`<br>`bit3: YAW_MODE`<br>`bit2&1: HORI_FRAME`<br>`bit0: STABLE_FLAG`|


## Broadcast Data Updates

### New

|CMD Set|CMD ID|Description|
|-------|------|-----------|
|0x02|0x03|Mission Status Push Info|
|0x02|0x04|Mission Events Push Info|

### Changed

|Struct Changed|2.3|3.1|
|--------------|---|---|
|Time Stamp|`uint32_t`|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`uint32_t time;`<br>&nbsp;&nbsp;`uint32_t asr_ts;`<br>&nbsp;&nbsp;`uint8_t sync_flag;`<br>`}sdk_time_stamp_t;`|
|Velocity|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`float32 x;`<br>&nbsp;&nbsp;`float32 y;`<br>&nbsp;&nbsp;`float z;`<br>&nbsp;&nbsp;`uint8_t health_flag:1;`<br>&nbsp;&nbsp;`uint8_t sensor_id:4;`<br>&nbsp;&nbsp;`uint8_t reserved:3;`<br>`} velocity_data_t;`|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`float32 x;`<br>&nbsp;&nbsp;`float32 y;`<br>&nbsp;&nbsp;`float z;`<br>&nbsp;&nbsp;`uint8_t health_flag:1;`<br>&nbsp;&nbsp;`uint8_t reserved:7;`<br>`} velocity_data_t;`|
|Ctrl Device|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`uint8_t cur_ctrl_dev_in_navi_mode: 3;`<br>&nbsp;&nbsp;`uint8_t serial_req_status: 1;`<br>&nbsp;&nbsp;`uint8_t reserved: 4;`<br>`} ctrl_device_t;`|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`uint8_t cur_api_ctrl_mode;`<br>&nbsp;&nbsp;`uint8_t cur_ctrl_dev_in_navi_mode: 3;`<br>&nbsp;&nbsp;`uint8_t serial_req_status: 1;`<br>&nbsp;&nbsp;`uint8_t vrc_enable_flag: 1;`<br>&nbsp;&nbsp;`uint8_t reserved: 3;`<br>`} ctrl_device_t;`|

## Library Updates

We have updated all libraries with new data structures and layer logic.

Concerning the maintance convenience, the new library is competiable with both 2.3 and 3.1 firmware.

Also, we keep the original 2.3 branch and old library as an archive for our old developers.

## `F` Mode Updates [IMPORTANT]

**Important: ** Developers are able to obtain the SDK control ability directly when the drone is powered on with mode bar in position `F`, while in 2.3 firmware, the same situation, develoeprs should switch the mode bar away then back to `F` in order to obtain control.

To avoid any dangerous situation, developers must pay attention and be careful on this change.

#固件与SDK接口更新（从2.3升级至3.1）

**注意：** 从2.3版本升级至3.1版本固件的开发者需要认真阅读此文档并按照接口的改动对自己的程序进行相应的改动，或直接使用最新的库文件替代2.3版本的库文件。

从3.1入手的开发者可以无视此文件直接阅读其他文档。

## 接口协议升级

### 新增接口

|命令集|命令码|描述|
|-------|------|-----------|
|0x01|0x05|锁定与解锁飞机|
|0x03||地面站控制相关指令|
|0x04|0x00|同步时间戳指令|
|0x05||虚拟遥控指令接口|

### 修改接口

|命令集|命令码|修改内容|2.3内容|3.1内容|
|-------|------|---|---|---|
|0x00|0x00|版本查询结果|2.3.10.0|3.1.10.0|
|0x00|0x01|取消了key的等级|`typedef struct{ `<br>&nbsp;&nbsp;`uint32_t app_id;`<br>&nbsp;&nbsp;`uint32_t app_level;`<br>&nbsp;&nbsp;`uint32_t app_version;`<br>&nbsp;&nbsp;`uint8_t app_bundle_id[32];`<br>`} sdk_activation_info_t;`|`typedef struct{ `<br>&nbsp;&nbsp;`uint32_t app_id;`<br>&nbsp;&nbsp;~~`uint32_t app_level;`~~<br>&nbsp;&nbsp;`uint32_t app_version;`<br>&nbsp;&nbsp;`uint8_t app_bundle_id[32];`<br>`} sdk_activation_info_t;`|
|0x00|0x01|激活时所填SDK版本|0x02030a00|0x03010a00|
|0x01|0x03|姿态控制标志位|`bit7&6: HORI_MODE`<br>`bit5&4: VERT_MODE`<br>`bit3: YAW_MODE`<br>`bit2&1: HORI_FRMAE`<br>`bit0: YAW_FRAME`|`bit7&6: HORI_MODE`<br>`bit5&4: VERT_MODE`<br>`bit3: YAW_MODE`<br>`bit2&1: HORI_FRMAE`<br>`bit0: STABLE_FLAG`|

## 推送数据更新

### 新增推送数据

|命令集|命令码|描述|
|------|------|----|
|0x02|0x03|地面站状态推送|
|0x02|0x04|地面站事件推送|

### 修改推送数据

|推送数据结构|2.3|3.1|
|------------|---|---|
|时间戳|`uint32_t`|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`uint32_t time;`<br>&nbsp;&nbsp;`uint32_t asr_ts;`<br>&nbsp;&nbsp;`uint8_t sync_flag;`<br>`}sdk_time_stamp_t;`|
|速度推送|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`float32 x;`<br>&nbsp;&nbsp;`float32 y;`<br>&nbsp;&nbsp;`float z;`<br>&nbsp;&nbsp;`uint8_t health_flag:1;`<br>&nbsp;&nbsp;`uint8_t sensor_id:4;`<br>&nbsp;&nbsp;`uint8_t reserved:3;`<br>`} velocity_data_t;`|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`float32 x;`<br>&nbsp;&nbsp;`float32 y;`<br>&nbsp;&nbsp;`float z;`<br>&nbsp;&nbsp;`uint8_t health_flag:1;`<br>&nbsp;&nbsp;`uint8_t reserved:7;`<br>`} velocity_data_t;`|
|控制设备|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`uint8_t cur_ctrl_dev_in_navi_mode: 3;`<br>&nbsp;&nbsp;`uint8_t serial_req_status: 1;`<br>&nbsp;&nbsp;`uint8_t reserved: 4;`<br>`} ctrl_device_t;`|`typedef struct`<br>`{`<br>&nbsp;&nbsp;`uint8_t cur_api_ctrl_mode;`<br>&nbsp;&nbsp;`uint8_t cur_ctrl_dev_in_navi_mode: 3;`<br>&nbsp;&nbsp;`uint8_t serial_req_status: 1;`<br>&nbsp;&nbsp;`uint8_t vrc_enable_flag: 1;`<br>&nbsp;&nbsp;`uint8_t reserved: 3;`<br>`} ctrl_device_t;`|

## 库文件升级

我们使用了新的数据结构与层逻辑来升级 Onboard SDK 的库文件。

出于技术支持与版本维护的方便性和可拓展性，我们决定新的 SDK 库不但支持最新的 3.1 固件，而且向下兼容 2.3 固件。

此外，我们仍然为之前的开发者们保留了原有的 2.3 分支（文档和库），供查阅与存档用。

## 档位模式升级 【重要】

**注意：** 在 3.1 固件中，若开机时模式开关被置于 `F` 档位，飞机将可以直接被 Onboard SDK 获取控制权。这不同于 2.3 中需要将 `F` 档位拨走再拨回来的设定。

开发者一定要注意此点，以防出现任何程序跑飞导致飞机暴走的危险。
