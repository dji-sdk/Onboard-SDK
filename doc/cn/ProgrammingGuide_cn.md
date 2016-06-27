#### 本文档已停止维护, 请移步到DJI开发者官网的 <https://developer.dji.com/onboard-sdk/documentation/application-development-guides/programming-guide.html> 查看最新版本. 

# Onboard SDK 编程指南
---
本文当中数据类型及函数在`DJI_Type.h`与`DJI_API.h`中实现。具体实现方式可参阅DJI_LIB中的源代码。  

---
## 配置环境

在开始使用我们提供的库之前，开发者需要在`DJI_HardDriver.h`中将 `HardDriver` 类继承，实现其虚函数表中的全部函数以及线程锁的lock及unlock函数。并创建一个新的API实例，传入刚才继承的HardDriver实例作为硬件接口。以及创建两个线程（或者一个），在while(1)函数体中枚举API实例中的sendPoll 和readPoll接口。

我们提供了 Windows 平台下的 QT 与 terminal 程序的例子。

## 回调机制
对于在开放协议中所有具有应答值的命令，在命令发送中均可通过如下的回调函数的方式获取相应的应答值。
以QT例程中的激活函数为例：
1、定义回调函数。

```c
static void activationCallback(CoreAPI* This, Header* header, UserData userData);
```

2、在调用激活函数时将回调函数名传入激活函数中。

```c
api->activate(&data, DJIonboardSDK::activationCallback, this);
```


3、各函数应答值(result)具体意义请参考[开放协议](OPENProtocol_cn.md#命令数据说明)中，各命令的应答值说明。

## 激活

```c
api->activate(&data, DJIonboardSDK::activationCallback, this);
```

## 获取/释放控制权

获取控制权时请确认：  

* 在PC assistant调参软件中，“启用API控制”勾选框已被勾选  
* 遥控器的模式选择开关已置于F档  

```c
api->setControl(true, DJIonboardSDK::setControlCallback, this);
```

## 自动起飞、降落及返航
该函数的回调函数中应答值对应[查询飞行状态切换结果](OPENProtocol_cn.md#命令码-0x02-查询飞行状态切换结果)的内容（下面的示例代码中未使用回调函数）。  

```c
flight->task(type);
```

## 姿态控制

我们建议开发者将姿态控制命令以50Hz的频率发送，用户可根据自己的开发环境通过如`usleep(20000)`、`ros::Duration(1/50)`等方式实现。  

姿态控制中具体被控制量与x，y，z，yaw的关系由控制模式字节(control_mode_byte)决定，更多内容请参阅[附录](Appendix_cn.md#模式标志字节说明)中“模式标志字节说明”部分。
我们建议开发者在水平方向上使用位置控制，具体使用说明请参考本文档[位置控制](#位置控制hori_pos)部分。速度及姿态控制飞控内部会完成闭环反馈，无需开发者在外部设计控制器。    
>备注：部分控制模式有进入条件限制：

>- 当且仅当GPS信号正常（health\_flag >=3）时，才可以使用水平*位置控制（HORI_POS）相关的控制指令
- 当GPS信号正常（health\_flag >=3），或者Gudiance系统正常工作（连接安装正确）时，可以使用水平*速度*控制（HORI_VEL）相关的控制指令


```c
FlightData data;
data.flag = flightFlag;
data.x = flightx;
data.y = flighty;
data.z = flightz;
data.yaw = flightyaw;
flight->setFlight(&data);
```

## 数据读取
如果希望读取飞控数据，请在调参软件中打开相对应的输出选项。同时请确认部分数据所在坐标系。
开发者在程序中需要声明一个相应的变量并使用对应的函数获取飞控的状态信息。
以获取姿态四元数为例：  
1、声明四元数结构体

```c
typedef struct QuaternionData
{
    float32_t q0;
    float32_t q1;
    float32_t q2;
    float32_t q3;
} QuaternionData;

QuaternionData q;
```

2、获取姿态四元数

```c
q = flight->getQuaternion()
```

飞控外发的其他数据类型及获取相应数据的函数请参考`DJI_Type.h`

```c
typedef struct BroadcastData
{
    unsigned short dataFlag;
    TimeStampData timeStamp;
    QuaternionData q;
    CommonData a;
    VelocityData v;
    CommonData w;
    PossitionData pos;
    MagnetData mag;
    RadioData rc;
    GimbalData gimbal;
    FlightStatus status;
    BatteryData battery;
    CtrlInfoData ctrlInfo;

    //! @note these variables are not send from FMU,
    //! just a record for user.
    uint8_t controlStatus;
    uint8_t activation;
} BroadcastData;

```

## GPS坐标解算
将经纬度换算成北东坐标系。（GPS经纬度为弧度值，北东坐标系单位为米）
实例中,`origin_longti`及`origin_lati`为原点位置经纬度，开发者可根据实际使用情况设定，建议采用飞行器起飞位置为原点位置；`longti`及`lati`为飞行器当前所在位置；`x`，`y`为解算出在北和东两个方向上相距原点的坐标，单位米。

~~~c
#define C_EARTH (double) 6378137.0
/* From GPS to Ground */
{
    double dlati = lati-origin_lati;
    double dlongti= longti-origin_longti;

    double x = dlati * C_EARTH;
    double y = dlongti * C_EARTH * cos(lati / 2.0 + origin_lati / 2.0);
}
~~~

## 姿态四元数的解算
将姿态四元数换算成Body系下的roll, pitch, yaw的弧度值。  
~~~c
    api_quaternion_data_t q;
    DJI_Pro_Get_Quaternion(&q);

    float roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
    float pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
    float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
~~~

## 位置控制(HORI_POS)
当水平方向采用位置控制模式（HORI_POS）时，水平方向上的输入量为当前位置与目标位置的偏移（offset）单位：米。  
实例中,采用Ground坐标系，`target`为目标位置坐标，`current`为飞行器位置坐标，开发者可通过GPS、Guidance或其他传感器计算出相关坐标信息。例如采用本文当中“GPS坐标解算“部分通过GPS解算位置信息。
由于飞控接受的最大控制频率为50 Hz，因此为保证控制效果，offset的计算频率应至少大于50 Hz。
~~~c
void update_offset()
{
    offset_x = target_x - current_x;
    offset_y = target_y - current_y;
}

/* Command thread */

FlightData data;
data.flag = 0x90;
data.x = offset_x;
data.y = offset_y;
data.z = target_z;
data.yaw = target_yaw;
flight->setFlight(&data);

~~~

