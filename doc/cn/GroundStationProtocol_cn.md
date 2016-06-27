#### 本文档已停止维护, 请移步到DJI开发者官网的 <https://developer.dji.com/onboard-sdk/documentation/introduction/ground-station-protocol.html> 查看最新版本. 

# 地面站接口说明

本文档旨在介绍地面站功能（航点任务、热点环绕任务、跟随任务）的协议格式与数据结构，在阅读之前推荐阅读[开放协议中的地面站部份](OPENProtocol_cn.md#命令集-0x03-地面站功能类)。

具体调用逻辑请参阅[地面站编程指南](GroundStationProgrammingGuide_cn.md)


## 命令集与命令码

<table>
    <tr>
        <th>命令集</th>
        <th>命令类型</th>
        <th>命令码</th>
        <th>说明</th>
    </tr>
    <tr>
        <th rowspan="19">0x03</th>
        <th rowspan="8">航点任务</th>
        <th>0x10</th>
        <th>上传航点任务中的任务信息</th>
    </tr>
    <tr>
        <th>0x11</th>
        <th>上传航点任务中的航点信息</th>
    </tr>
    <tr>
        <th>0x12</th>
        <th>开始或结束航点任务</th>
    </tr>
    <tr>
        <th>0x13</th>
        <th>暂停或恢复航点任务</th>
    </tr>
    <tr>
        <th>0x14</th>
        <th>下载航点任务的任务信息</th>
    </tr>
    <tr>
        <th>0x15</th>
        <th>下载航点任务的具体航点信息</th>
    </tr>
    <tr>
        <th>0x16</th>
        <th>设置航点任务的巡航速度</th>
    </tr>
    <tr>
        <th>0x17</th>
        <th>读取航点任务的巡航速度</th>
    </tr>
    <tr>
        <th rowspan="8">热点环绕任务</th>
        <th>0x20 </th>
        <th>上传热点环绕任务信息并开始任务</th>
    </tr>
    <tr>
        <th>0x21</th>
        <th>终止热点环绕任务</th>
    </tr>
    <tr>
        <th>0x22</th>
        <th>暂停或恢复热点环绕任务</th>
    </tr>
    <tr>
        <th>0x23</th>
        <th>设置热点环绕任务的巡航速度</th>
    </tr>
    <tr>
        <th>0x24</th>
        <th>设置热点环绕任务的任务半径</th>
    </tr>
    <tr>
        <th>0x25</th>
        <th>重制热点环绕任务的偏航角</th>
    </tr>
    <tr>
        <th>0x26</th>
        <th>下载热点环绕任务的任务信息</th>
    </tr>
    <tr>
        <th>0x27</th>
        <th>启动自动半径模式</th>
    </tr>
    <tr>
        <th rowspan="4">跟随任务</th>
        <th>0x30</th>
        <th>上传跟随任务的任务信息并开始任务</th>
    </tr>
    <tr>
        <th>0x31</th>
        <th>停止跟随任务</th>
    </tr>
    <tr>
        <th>0x32</th>
        <th>暂停或恢复跟随任务</th>
    </tr>
    <tr>
        <th>0x33</th>
        <th>更新跟随任务的跟随点信息</th>
    </tr>
    <tr>
        <th>0x02</th>
        <th>任务类型推送信息</th>
        <th>0x03</th>
        <th>当前任务类型信息</th>
    </tr>
    <tr>
        <th>0x02</th>
        <th>任务事件推送信息</th>
        <th>0x04</th>
        <th>任务事件</th>
    </tr>
</table>

## 数据结构

**注意：** 地面站相关的数据结构，所有保留字节必需设定成0。不然会产生不必要的错误。

--- 

### 0x03, 0x10：上传航点任务的任务信息

请求数据结构：
```c
typedef struct {

    uint8_t length;//航点总数
    float vel_cmd_range;//摇杆控制速度上限（2~15m）
    float idle_vel;//不通过遥控进行速度干涉时的某人巡航速度
    uint8_t action_on_finish;//任务结束后的动作
                            //0: 无动作
                            //1: 返航
                            //2: 自动降落
                            //3: 返回至第一个（index 为 0）的航点
                            //4: 无限循环航点任务
    uint8_t mission_exec_num;//任务运行次数
                            //1: 一次
                            //2: 两次
    uint8_t yaw_mode;//偏航角控制模式
                    //0: 机头永远指向下一个航点
                    //1: 机头永远保持初始时的方向
                    //2: 机头方向又遥控器控制
                    //3: 使用航点中的偏航信息
    uint8_t trace_mode;//飞行模式
                    //0: 点对点飞行，支持在航点上进行航点动作
                    //1: 曲线轨迹优化飞行，不支持航点动作
    uint8_t action_on_rc_lost;//失去遥控信号时的动作
                            //0: 推出航点并进入失控保护逻辑
                            //1: 继续当前航点任务
    uint8_t gimbal_pitch_mode;//云台俯仰控制
                            //0: 自由模式，不控云台俯仰
                            //1: 自动模式
    double hp_lati;//关注点纬度 (弧度)
    double hp_longti;//关注点经度 (弧度)
    float hp_alti;//关注点高度 (与起飞点的相对高度)
    uint8_t resv[16];//保留字节，必需全部置为0
    
}waypoint_mission_info_comm_t;
```

返回数据类型：`uint8_t`。

**注意：** 此处飞控对于航点上传信息的返回值恒为0，无论成功与否。用户需要自己检查航点信息是否合法，若不合法，再下一步上传每一个航点任务信息时会返回0xEA，航点任务信息尚未上传。

### 0x03, 0x11：上传航点任务的具体航点信息（需要提供索引值）

请求数据结构：
```c
typedef struct {
    double latitude;//航点纬度 (弧度)
    double longitude;//航点经度 (弧度)
    float altitude;//航点高度 (与起飞点的相对高度)
    float damping_dis;//转弯半径 (只在曲线轨迹优化模式下生效)
    int16_t tgt_yaw;//飞到航点时的机头朝向 (角度)
    int16_t tgt_gimbal_pitch;//飞到航点时的云台俯仰角
    uint8_t turn_mode;//转弯模式
                    //0: 顺时针
                    //1: 逆时针
    uint8_t resv[8];//保留字节
    
    uint8_t has_action;//航点动作标志位
                    //0: 无动作
                    //1: 有动作
    uint16_t action_time_limit;//航点动作运行事件限制（单位：秒）
    waypoint_action_comm_t action;//航点动作信息

} waypoint_comm_t

```

返回数据类型：
```c
struct waypoint_upload_ack{
    uint8_t ack;
    uint8_t index;
};
```
    

航点任务结构体：

```c
typedef struct {
    uint8_t action_num :4;//航点数量
    uint8_t action_rpt :4;//航点运行次数
    
    uint8_t command_list[15];//航点任务队列。最多支持15个
    int16_t command_param[15];//任务参数队列，最多支持15个

}waypoint_action_comm_t
```

一共有总共六种任务类型，需要填写在`command_list`中：


|任务类型|值|对应参数|说明|
|----|------|--------|----|
|WP_ACTION_STAY|0|时间（毫秒）|悬停|
|WP_ACTION_SIMPLE_SHOT|1|不适用|拍照|
|WP_ACTION_VIDEO_START|2|不适用|开始录像|
|WP_ACTION_VIDEO_STOP|3|不适用|停止录像|
|WP_ACTION_CRAFT_YAW|4|偏航 (-180~180)|调整机头方向|
|WP_ACTION_GIMBAL_PITCH|5|云台俯仰|调整云台俯仰（0：平视， -90： 竖直向下）|

**注意：** 当所有航点信息上传完成后，飞机才会对所有航点进行信息合法的校验。即：若所有航点中至少有一个航点有错，在切只在上传最后一个航点信息的回调中才会出新错误码，而此错误码并不一定是指代最后一个航点信息是错误的。


### 0x03, 0x12：开始或停止航点任务

请求数据结构：

```c
uint8_t start;//0-> 开始, 1-> 停止
```

返回数据类型： `uint8_t`

### 0x03, 0x13：暂停或恢复航点任务

请求数据结构：

```c
uint8_t pause;//0-> 暂停, 1-> 恢复
```

返回数据类型：`uint8_t`


### 0x03, 0x14：下载航点任务的任务信息

请求数据结构:

`uint8_t` 任意内容

返回数据类型：

```c
struct waypoint_task_download_ack{
    uint8_t ack;
    waypoint_mission_info_comm_t wp_task;//defined in previous part
};
```

### 0x03, 0x15：下载航点任务的具体航点信息

请求数据结构：

```c
uint8_t index;//航点索引
```

返回数据类型：

```c
struct waypoint_download_ack{
    uint8_t ack;
    uint8_t index;//航点索引
    waypoint_comm_t wp_data;//航点信息结构体
};
```

### 0x03, 0x16：设置巡航速度

请求数据结构：

```c
float idle_veloctity;
```

返回数据类型：

```c
struct waypoint_set_vel_ack{
    uint8_t ack;
    float idle_velcity;
};
```

### 0x03, 0x17：读取巡航速度

请求数据结构：

`uint8_t` 任意值

返回数据类型：

```c
struct waypoint_read_vel_ack{
    uint8_t ack;
    float idle_velcity;
};
```


### 0x03, 0x20：上传热点环绕任务并开始

请求数据类型：

```c
typedef struct{
    uint8_t version;//保留，设为0
    double hp_latitude;//热点纬度（弧度）
    double hp_longitude;//热点经度（弧度）
    double hp_altitude;//热点高度（与起飞点的相对高度）
    double hp_radius;//环绕半径（5米～500米）
    float angle_rate;//角速度（0~30读/秒）
    uint8_t is_clockwise;// 0->逆时针环绕， 1->顺时针环绕
    uint8_t start_point_position;//起始点位置
                                //0：最北环绕点
                                //1：最南环绕点
                                //2：最西环绕点
                                //3：最东环绕点
                                //4：与当前位置最近的环绕点
    uint8_t yaw_mode;//偏航模式
                    //0：机头方向与速度方向保持一致
                    //1：机头向内
                    //2：机头向外
                    //3：机头方向由遥控器控制
                    //4：机头方向保持开始时的角度
    
    uint8_t reserved[11];//保留字节

} hotpoint_mission_setting_t;
```

返回数据类型：

```c
struct hotpoint_upload_ack{
    uint8_t ack;
    float max_radius; //最大环绕半径
};
```

### 0x03, 0x21：停止热点任务

请求数据结构：

`uint8_t` 任意值

返回数据类型：`uint8_t`

### 0x03, 0x22：暂停或恢复热点环绕任务

请求数据结构：

```c
uint8_t pause; //0->暂停, 1->恢复
```

返回数据类型：`uint8_t`

### 0x03, 0x23：设置热点环绕巡航速度

```c
typedef struct{
    uint8_t is_clockwise;
    float idle_velocity;
}hotpoint_set_vel_t;
```

返回数据类型：`uint8_t`

### 0x03, 0x24：设置环绕半径

请求数据结构：

```c
float radius;
```

返回数据类型：`uint8_t`

### 0x03, 0x25：重制设点环绕偏航角

请求数据结构：

`uint8_t` 任意值

返回数据类型：`uint8_t`

### 0x03, 0x26：下载热点环绕任务信息

请求数据结构：

`uint8_t` 任意值

返回数据类型：

```c
struct hotpoint_download_ack {
    uint8_t ack;
    hotpoint_mission_setting_t hotpoint_task;//热点环绕任务结构体
};
```

### 0x03, 0x27：启用自动半径模式

请求数据结构：

```c
struct hotpoint_auto_radiu {
    uint8_t on_off; //1->enable, 0->disable
    int8_t rate;//radiu change rate
};
```

返回数据类型：`uint8_t`


### 0x03, 0x30：上传跟随任务并开始

请求数据结构：

```c
typedef struct{
    uint8_t follow_mode;//保留，设为0
    uint8_t yaw_mode;//偏航角模式
                    //0：由遥控器控制
                    //1：指向被跟随物体
    double init_latitude;//被跟随物体初始位置纬度（弧度）
    double init_longitude;//被跟随物体初始位置经度（弧度）
    uint16_t init_alti;//被跟随物体初始高度
    uint16_t init_mag_angle;//保留
    uint8_t follow_sensitivity;//保留。设为0
}follow_me_mission_setting_t;
```

返回数据类型：`uint8_t`

### 0x03, 0x31：停止跟随任务

请求数据结构：

`uint8_t` 任意值

返回数据类型：`uint8_t`

### 0x03, 0x32：暂停或恢复跟随任务

请求数据结构：

```c
uint8_t pause; //0->暂停， 1->恢复
```

返回数据类型：`uint8_t`

### 0x03, 0x33：更新被跟随目标位置信息

请求数据结构：
 
```c
typedef struct{
    double latitude; //被跟随物体的纬度（弧度）
    double longitude;//被跟随物体的经度（弧度）
    uint16_t altitude;//被跟随物体的高度（相对于起飞点的高度）
    uint16_t mag_angle;//保留

}cmd_mission_follow_target_info;
```

没有应答值数据返回

---

注意：以下为飞控推送信息而非指令接口，其调用方法类似于飞机的状态广播信息。在使用之前开发者需要在调参软件中勾选地面站信息推送。

    ![](Images/groundstation.png)

### 0x02, 0x03：当前任务状态推送信息

一共有四种推送状态的类型共用此命令集和命令码，他们的结构体有相同长度。

开发者可以通过他们的第一个byte，`mission_type`，来判断是哪种类型。

`mission_type`定义如下：

~~~c
typedef enum
{
    NAVI_MODE_ATTI,
    NAVI_MISSION_WAYPOINT,
    NAVI_MISSION_HOTPOINT,
    NAVI_MISSION_FOLLOWME,
    NAVI_MISSION_IOC,
}navi_type;
~~~

航点任务状态推送信息：

```c
typedef struct{
    uint8_t mission_type; //NAVI_MISSION_WAYPOINT
    uint8_t target_waypoint; //当前目标航点的索引值
    uint8_t current_state;//当前任务状态
    uint8_t error_notification;//错误信息
    uint16_t reserved;//保留

} cmd_mission_waypoint_status_push_t;
```

热点环绕任务状态推送信息：

```c
typedef struct{
    uint8_t mission_type;//NAVI_MISSION_HOTPOINT
    uint8_t mission_status;//任务状态
                        //0：初始化
                        //1：正在运行
                        //2：已停止
    uint16_t hp_exec_radius; //与热点的距离，单位厘米
    uint8_t reason; 
    uint8_t hp_exec_vel;//角速度。单位度x10
} cmd_mission_hotpoint_status_push_t;
```

跟随任务状态推送信息：

```c
typedef struct{
    uint8_t mission_type;//NAVI_MISSION_FOLLOWM
    uint8_t reserved_1;
    uint16_t reserved_2;
    uint16_t reserved_3;
}cmd_mission_folowme_status_push_t;
```

通用任务状态推送信息(NAVI_MODE_ATTI & NAVI_MISSION_IOC)：

```c
typedef struct{
    uint8_t mission_type;
    uint8_t last_mission_type;
    uint8_t is_broken :1;
    uint8_t reserved_1 :7;
    uint8_t reason;
    uint8_t reserved_2;
    uint8_t reserved_3;
}cmd_mission_default_status_push_t;
```

### 0x02, 0x04 航点任务事件推送信息：

航点任务事件推送信息共有三种类型，他们的结构体拥有相同长度。

开发者可以通过第一个字节，`incident_type`，来区分类型。

`incident_type`.定义如下：

~~~c
typedef enum
{
    NAVI_UPLOAD_FINISH,
    NAVI_MISSION_FINISH;
    NAVI_MISSION_WP_REACH_POINT,
}incident_type;
~~~

航点任务上传事件推送

```c
typedef struct{
    uint8_t incident_type;
    uint8_t is_mission_valid;
    uint16_t estimated_run_time;
    uint16_t reserved;
}cmd_mission_wp_upload_incident_t;
```

航点任务结束事件推送

```c
typedef struct{
    uint8_t incident_type;
    uint8_t repeat;
    uint16_t reserved_1;
    uint16_t reserved_2;
}cmd_mission_wp_finish_incident_t;
```

航点任务航点到达事件推送：

```c
typedef struct{
    uint8_t incident_type;
    uint8_t waypoint_index;
    uint8_t current_state;
    uint8_t reserved_1;
    uint8_t reserved_2;
}cmd_mission_wp_reached_incident_t;
```

## 返回码
<table>
    <tr>
        <th colspan="4">公共返回码</th>
    </tr>
    <tr>
        <th>0x00</th>
        <th>成功</th>
        <th>0xD0</th>
        <th>模式开关未在F档</th>
    </tr>
    <tr>
        <th>0xD1</th>
        <th>导航模式未开启</th>
        <th>0xD2</th>
        <th>IOC开启</th>
    </tr>
    <tr>
        <th>0xD3</th>
        <th>任务没有初始化</th>
        <th>0xD4</th>
        <th>没有运行该任务</th>
    </tr>
    <tr>
        <th>0xD5</th>
        <th>正在运行该任务</th>
        <th>0xD6</th>
        <th>飞行时间无法满足</th>
    </tr>
    <tr>
        <th>0xD7</th>
        <th>有更高优先级的任务正在运行</th>
        <th>0xD8</th>
        <th>GPS信号强度无法满足</th>
    </tr>
    <tr>
        <th>0xD9</th>
        <th>电池电量过低</th>
        <th>0xDA</th>
        <th>初始化时飞机不在空中</th>
    </tr>
    <tr>
        <th>0xDB</th>
        <th>无效的任务参数</th>
        <th>0xDC</th>
        <th>执行条件不满足</th>
    </tr>
    <tr>
        <th>0xDD</th>
        <th>任务可能跨域禁飞区</th>
        <th>0xDE</th>
        <th>HOME点尚未记录</th>
    </tr>
    <tr>
        <th>0xDF</th>
        <th>飞机已处于限飞区或禁飞缓冲区</th>
        <th>0xC0</th>
        <th>高度太高</th>
    </tr>
    <tr>
        <th>0xC1</th>
        <th>高度太低</th>
        <th>0xC7</th>
        <th>限远约束</th>
    </tr>
    <tr>
        <th>0xC8</th>
        <th>产品不支持地面站</th>
        <th>0xC9</th>
        <th>飞机距离热点或第一个航点太远</th>
    </tr>
    <tr>
        <th>0xCA</th>
        <th>当前处于新手模式</th>
        <th>0xF0</th>
        <th>飞机正在起飞</th>
    </tr>
    <tr>
        <th>0xF1</th>
        <th>飞机正在着陆</th>
        <th>0xF2</th>
        <th>飞机正在返航</th>
    </tr>
    <tr>
        <th>0xF3</th>
        <th>飞机正在启动电机</th>
        <th>0xF4</th>
        <th>命令错误</th>
    </tr>
    <tr>
        <th>0xFF</th>
        <th>未知错误</th>
    </tr>
    <tr>
        <th colspan="4">跟随任务返回码</th>
    </tr>
    <tr>
        <th>0xB0</th>
        <th>初始化时飞机与手机距离过远</th>
        <th>0xB1</th>
        <th>断线时间过长</th>
    </tr>
    <tr>
        <th>0xB2</th>
        <th>初始化时云台俯仰角过大</th>
    </tr>
    <tr>
        <th colspan="4">热点任务返回码</th>
    </tr>
    <tr>
        <th>0xC2</th>
        <th>无效的半径</th>
        <th>0xC3</th>
        <th>速度过大</th>
    </tr>
    <tr>
        <th>0xC4</th>
        <th>切入点非法</th>
        <th>0xC5</th>
        <th>偏航模式非法</th>
    </tr>
    <tr>
        <th>0xC6</th>
        <th>距离轨迹太远无法恢复</th>
        <th>0xA2</th>
        <th>浮点数参数非法</th>
    </tr>
    <tr>
        <th>0xA3</th>
        <th>无效的经纬度</th>
        <th>0xA6</th>
        <th>无效的方向参数</th>
    </tr>
    <tr>
        <th>0xA9</th>
        <th>热点已经暂停</th>
        <th>0xAA</th>
        <th>热点没有暂停</th>
    </tr>
    <tr>
        <th colspan="4">航点任务返回码</th>
    </tr>
    <tr>
        <th>0xE0</th>
        <th>航点任务信息不合法</th>
        <th>0xE1</th>
        <th>航点信息不合法</th>
    </tr>
    <tr>
        <th>0xE2</th>
        <th>任务距离过长</th>
        <th>0xE3</th>
        <th>整体飞行距离过长</th>
    </tr>
    <tr>
        <th>0xE4</th>
        <th>索引超过最大行点数</th>
        <th>0xE5</th>
        <th>相邻航点距离太近</th>
    </tr>
    <tr>
        <th>0xE6</th>
        <th>相邻航点距离太远</th>
        <th>0xE7</th>
        <th>damping检查失败</th>
    </tr>
    <tr>
        <th>0xE8</th>
        <th>动作参数不合法</th>
        <th>0xE9</th>
        <th>航点还未上传完成</th>
    </tr>
    <tr>
        <th>0xEA</th>
        <th>任务信息还未上传完成</th>
        <th>0xEB</th>
        <th>航点还未完全上传</th>
    </tr>
    <tr>
        <th>0xEC</th>
        <th>请求状态正在执行</th>
        <th>0xED</th>
        <th>并没有执行航点功能，无法暂停</th>
    </tr>
    <tr>
        <th colspan="4">IOC返回码</th>
    </tr>
    <tr>
        <th>0xA0</th>
        <th>距离HOME点太近</th>
        <th>0xA1</th>
        <th>IOC类型错误</th>
    </tr>
    
</table>
