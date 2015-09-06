# DJI Onboard API 编程指南

*如发现任何错误，请通过Github issue或开发者论坛或邮件反馈给我们。欢迎提交pull request来帮助我们修正问题，关于文档的修改需要符合[格式标准](https://github.com/dji-sdk/Onboard-SDK/issues/19)*

---

DJI 为开发者提供两种功能完善的飞行控制 API 帮助开发飞行应用：Mobile API 和 Onboard API。Mobile API 是 DJI Mobile SDK 的核心部分，开发者可以基于 iOS/Android 系统上的 SDK
库编写控制飞行器的移动端应用。而 Onboard API 则提供串行接口（UART），允许开发者将 自己的计算设备挂载到飞行器上，通过有线的方式直接控制飞行器。 

本指南中介绍了如何通过程序与 MATRICE 100 交互并发送控制指令。我们推荐开发者先通过快速入门实现我们的示例代码，然后再阅读编程指南。
 
## 协议说明 

### 协议格式

   ```
   |<--------------Protocol Frame Header---------------->|<--Protocol Frame Data-->|<--Protocol Frame Checksum-->|
   |SOF|LEN|VER|SESSION|A|RES0|PADDING|ENC|RES1|SEQ|CRC16|          DATA           |            CRC32            |
   ``` 

### 协议格式说明

<table>
<tr>
  <th>字段</th>
  <th>字节索引</th>
  <th>大小（单位 bit）</th>
  <th>说明</th>
</tr>

<tr>
  <td>SOF</td>
  <td>0</td>
  <td>8</td>
  <td>帧起始标识。固定位 0xAA</td>
</tr>

<tr>
  <td>LEN</td>
  <td rowspan="2">1</td>
  <td>10</td>
  <td>帧长度。最大为 1023 bytes</td>
</tr>

<tr>
  <td>VER</td>
  <td>6</td>
  <td>协议版本</td>
</tr>

<tr>
  <td>SESSION</td>
  <td rowspan="3">3</td>
  <td>5</td>
  <td>通信过程中的会话 ID</td>
</tr>

<tr>
  <td>A</td>
  <td>1</td>
  <td>帧标识<ol start="0"><li>数据帧</li><li>应答帧</li></ol></td>
</tr>

<tr>
  <td>RES0</td>
  <td>2</td>
  <td>保留不用。固定值为 0x0</td>
</tr>

<tr>
  <td>PADDING</td>
  <td rowspan="2">4</td>
  <td>5</td>
  <td>加密帧数据时附加的数据长度</td>
</tr>

<tr>
  <td>ENC</td>
  <td>3</td>
  <td>帧数据加密类型<ol start="0"><li>不加密</li><li>AES 加密</li></ol></td>
</tr>

<tr>
  <td>RES1</td>
  <td>5</td>
  <td>24</td>
  <td>保留不用。固定值为 0x0</td>
</tr>

<tr>
  <td>SEQ</td>
  <td>8</td>
  <td>16</td>
  <td>帧序列号</td>
</tr>

<tr>
  <td>CRC16</td>
  <td>10</td>
  <td>16</td>
  <td>帧头 CRC16 校验值</td>
</tr>

<tr>
  <td>DATA</td>
  <td>12</td>
  <td>---</td>
  <td>帧数据段。最大长度为 1007bytes</td>
</tr>

<tr>
  <td>CRC32</td>
  <td>---</td>
  <td>32</td>
  <td>帧 CRC32 校验值</td>
</tr>
</table>

DATA 长度大小不固定，最大长度为 1007。crc32字节索引根据 DATA 长度大小而定。

### 协议数据段说明

飞控和机载设备通信的数据包分为三类：

1. 命令数据包。从机载设备发送到飞控，包含对飞行器的控制指令。
2. 信息数据包。从飞控发送到机载设备，包含飞控的各种状态信息和传感器数据。
3. 应答数据包。从飞控发送到机载设备，包含控制指令的执行结果。

#### 机载设备发送给飞控的命令数据包的数据段格式 

```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|字段|字节索引|大小（单位 byte）|说明|
|----|--------|-----------------|----|
|COMMAND SET|0|1|命令集|
|COMMAND ID|1|1|命令码|
|COMMAND DATA|2|大小根据具体命令而定|命令数据|

#### 飞控发给机载设备的信息数据包的数据段格式 

```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|字段|字节索引|大小（单位 byte）|说明|
|----|--------|-----------------|----|
|COMMAND SET|0|1|命令集|
|COMMAND ID|1|1|命令码|
|COMMAND DATA|2|大小根据具体命令而定|飞机状态及传感器等数据|

#### 飞控发给机载设备的应答数据包的数据段格式 

```
|<-Protocol Frame Data->|
|COMMAND RETURN|ACK DATA|
```

|字段|字节索引|大小（单位 byte）|说明|
|----|--------|-----------------|----|
|COMMAND RETURN|0|2|命令执行的返回信息|
|ACK DATA|2|大小根据具体命令而定。|应答数据|

### 通信会话机制

协议设计使用了会话机制，以保证命令数据和应答数据不会因为丢包而出现通信双方异常。通信双方在向对方发起通信会话时，可以根据需要通过设置协议的 SESSION 字段来选择会话方式。协议中设计了三种会话方式。

|会话方式|SESSION|描述|
|--------|-------|----|
|方式1|0|发送端不需要接收端应答|
|方式2|1|发送端需要接收端应答数据，但是可以容忍应答数据丢包|
|方式3|2-31|发送端需要正确收到接收端的应答包。<br>发送端使用这些session 发送命令数据包时，接收端应答后要保存当前的应答包作为该 session 的应答数据，应答包中包含该命令数据包中的 sequence number 和 session id。如果通信过程中，发送端没有正确收到应答包，可以重新发送该命令数据包，接收端收到后将保存的应答包重新发送回去。<br>下一次，如果发送端使用和上一次相同的 session id，但不同的 sequence number 来发送命令数据包时，接收端会丢弃上一次保存的 session 应答数据，重新保存新的 session 应答数据。|

### API 示例

假设使用以下 C/C++枚举类型表示会话方式：

~~~c
enum SESSION_MODE {
  SESSION_MODE1,
  SESSION_MODE2,
  SESSION_MODE3
}
~~~

命令数据包的回调接口函数定义如下：

    typedef void (*CMD_CALLBACK_FUNC)(const void* p_data, unsigned int n_size)
    
假设通信中发送协议数据的函数定义如下：

    unsigned int Linklayer_Send(SESSION_MODE session_mode, const void* p_data, unsigned int n_size, char enc_type, unsigned short ack_timeout, unsigned char retry_time, CMD_CALLBACK_FUNC cmd_callback)
    
参数 session_mode：会话方式。<br>
参数 p_data：指向待发协议数据流的起始地址。<br>
参数 n_size：协议数据流的大小。<br>
参数 enc_type：是否采用加密发送。<br>
参数 ack_timeout：使用会话方式 3 时接收端应答的超时时间，单位 ms。<br>
参数 retry_time:使用会话方式 3 接收端不应答时，发送端重发的次数。<br>
参数 cmd_callback:回调函数接口。<br>

**备注：由于会话方式 3 是一种可靠会话方式，开发者在协议链路层实现中应考虑数据丢包后的重发机制，在设计链路层发送接口时应提供超时时间、重发次数等参数。**

---

## 命令集说明 

### 命令及权限

DJI Onboard API 相关的命令分为三大类：

|类别|说明|命令集代码|
|----|----|----------|
|激活验证类|该命令集包含的 ID 只与激活相关|0x00|
|飞控接受的控制控制数据的命令集|0x01|
|飞控外发的数据|飞控外发的命令集|0x02|

每类命令有唯一的命令集代码，命令集包含的所有命令有各自的命名码和命令数据。飞控接受的控制命令全部有权限级别。在未来版本中会有更多的控制命令以不同的权限级别开放。目前权限级别如下：

|权限级别（API 级别）|权限描述|
|--------------------|--------|
|0|API 激活命令|
|1|相机和云台的控制命令|
|2|飞行控制命令|

### 命令集

#### 命令集 0x00 激活验证类 

API 激活验证命令集的所有命令权限级别为 0，即所有用户都可以使用命令集中的命令对飞机进行激活与版本查询等操作。激活 API 通过 DJI Pilot 与 DJI Server 连接，需要手机连接互联网。

##### 命令码 0x00 获取 API 版本

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td>请求数据</td>
  <td>1</td>
  <td>1</td>
  <td>任意请求数据</td>
</tr>

<tr>
  <td rowspan="3">应答数据</td>
  <td>0</td>
  <td>2</td>
  <td>返回码<br>0x0000：激活成功<br>0xFF00：命令不支持<br>0xFF01：机载设备无授权<br>0xFF02：机载设备权限不足</td>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>CRC32, 字符串版本的 CRC32</td>
</tr>

<tr>
  <td>6</td>
  <td>32</td>
  <td>不定长，最大长度 32bytes。有效部分到’\0’结尾</td>
</tr>
</table>

推荐接收应答数据的 C/C++结构体：

~~~c
typedef struct {
  unsigned short version_ack;
  unsigned int varsion_crc;
  signed char version_number[32];
} version_query_data_t;
~~~

假设获取 API 版本命令的回调函数为：

~~~c
void print_sdk_version(const void* p_data, unsigned int n_size) {
  version_quesry_data_t* p_version = (version_query_data_t*)p_data;
  if (p_version->version_ack == 0) {
    printf("%s\n",p_version->version_name);
  }
}
~~~

应用程序中发送请求获取 API 版本命令的操作如下：

~~~c
unsigned char cmd_buf[3];
cmd_buf[0] = 0x00; //command set
cmd_buf[1] = 0x00; //command id
cmd+buf[2] = 0x00; //command data, an arbitrary number as said above
Linklayer_Send(SESSION_MODE3,
                cmd_buf,
                3,
                0,
                200.
                3,
                print_sdk_version
};
~~~  

以上使用的会话方式 3 进行 API 版本获取请求，飞机收到请求并响应后，应用程序相应的回调函数 print_sdk_version 会执行，并输出版本信息示例如下：

    SDK vX.X XXXX

###### 命令码 0x01 激活 API

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td rowspan="4">请求数据</td>
  <td>0</td>
  <td>4</td>
  <td>appid, 服务器注册时候生成的内容</td>
</tr>

<tr>
  <td>4</td>
  <td>4</td>
  <td>api_level，API 权限级别</td>
</tr>

<tr>
  <td>8</td>
  <td>4</td>
  <td>app_ver，用户程序版本</td>
</tr>

<tr>
  <td>12</td>
  <td>32</td>
  <td>bundle_id， App 的唯一 ID</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>返回码，应答码： <ol start="0"><li>成功</li><li>参数非法，参数长度不匹配</li><li>数据包加密了，未能正确识别</li><li>没有激活过的设备，尝试激活</li><li>DJI App 没有响应，可能是没有连接 DJI App</li><li>DJI App 没有联网</li><li>服务器拒绝，激活失败</li><li>权限级别不够</li></ol></td>
</tr>

</table>

推荐发送命令数据的 C/C++结构体

~~~c
typedef __attribute_((__packed__)) struct { //1 byte aligned
  unsigned int app_id;
  unsigned int ap_api_level;
  unsigned int app_ver;
  unsigned char app_bundle_id[32];
} activation_data_t;
~~~

**备注：文档中介绍的结构体示例都要求 1 字节对齐。开发者需要根据自身的开发编程环境及编程语言保证结构体的对齐方式为 1 字节对齐。**

推荐接收应答数据的 C/C++枚举类型为：

~~~c
enum ErrorCodeForActivatie {
  errActivateSuccess,
  errActivateInvalidParamLength,
  errActivateDataIsEncrypted,
  errActivateNewDevice,
  errActivateDJIAppNotConnected.
  errActivateDJIAppNoInternet,
  errActivateDJIServerReject,
  errActivateLevelError
};
~~~

假设激活 API 命令的回调函数为：

~~~c
void activation_callback(const void* p_data, unsigned int n_size) {

}
~~~

应用程序中发送激活 API 命令的操作如下：

~~~c
unsigned char com_buf[46];
activation_data_t activation_request_data;
//USER TODO...
//activation_request_data.app_id        =0x00;
//activation_request_data.app_api_level =0x00;
//activation_request_data.app_ver       =0x00;
//memset(activation_request_data.app_bundle_id,0,32)
cmd_buf[0] = 0x00; //command set
cmd_buf[1] = 0x01; //command id
memcpy((void*)&cmd_buf[2], (void*)&activation_request_data),sizeof(activation_data_t));
Linklayer_Send(SESSION_MODE3,
                cmd_buf,
                46,
                0,
                200,
                3,
                activation_callback
  );
~~~

以上使用的会话方式 3 进行激活 API 请求，飞机收到请求并响应后，应用程序相应的回调函数 activation_callback 会执行，可以判断是否激活成功。

###### 命令码 0xFE 透传数据（机载设备至移动设备）

机载设备发送给移动的数据包。最大包大小为 100 字节，带宽约 8KB/s。

|数据类型|偏移（字节）|大小（字节）|说明|
|--------|------------|------------|----|
|请求数据|0|1~100|用户自定义数据|
|应答数据|0|2|返回码，应答码 0：成功|

~~~c
char cmd_buf[10];
cmd_buf[0] = 0x00;
cmd_buf[1] = 0xFE;
memcpy(&cmd_buf[2], "Hello!", 7);
Linklayer_Send(SESSION_MODE3,
                cmd_buf,
                9,
                0,
                200,
                3,
                0
);
~~~

#### 命令集 0x01 飞行控制类 

##### 命令码 0x00 请求获得控制权

|数据类型|偏移（字节）|大小（字节）|说明|
|--------|------------|------------|----|
|请求数据|0|1|<ul><li>1 = 请求获得控制权</li><li>0 = 请求释放控制权</li></ul>|
|应答数据|0|2|返回码 <ul><li>0x0001：成功释放控制权</li><li>0x0002：成功获得控制权</li><li>0x0003：获得控制权失败</li></ul>

飞机可以接受三种设备的控制输入：遥控器、移动设备、机载设备而。三种设备的控制输入的优先级最大是遥控器，其次是移动设备，优先级最低是机载设备。假设请求获得控制权命令的回调函数为：

~~~c
void get_control_callback(const void* p_data, unsigned int n_size) {

}
~~~

应用程序中发送激活 API 命令的操作如下：

~~~c
unsigned char cmd_buf[46];
cmd_buf[0] = 0x01; //command set
cmd_buf[1] = 0x00; //command id
cmd_buf[2] = 0x01; //get control
Linklayer_send(SESSION_MODE3,
                cmd_buf,
                3,
                1,
                200,
                3,
                get_control_callback
);
~~~

以上使用的会话方式 3 进行获取控制权请求，飞机收到请求并响应后，应用程序相应的回调函数 get_control_callback 会执行，可以判断是否成功。

**备注：获得控制权请求需在激活成功后进行，激活成功后，机载设备和飞机的通信必须采用密文通信。**

##### 命令码 0x01-0x02 状态控制命令

机载设备对飞机的状态控制分为两个阶段。

第一个阶段是发送命令码为 0x01 的状态控制指令。

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td rowspan="2">请求数据</td>
  <td>0</td>
  <td>1</td>
  <td>指令序列号</td>
</tr>

<tr>
  <td>1</td>
  <td>1</td>
  <td>控制<ui><li>1 = 请求进入自动返航</li><li>4 = 请求自动起飞</li><li>6 = 请求自动降落</li></ui></td>
</tr>

<tr>
  <td>应答数据</td>
  <td>0</td>
  <td>1</td>
  <td>返回码<ui><li>0x0001：执行失败</li><li>0x0002：开始执行</li></ui></td>
</tr>

</table>

飞机收到状态控制指令之后会立即发送表明已经收到指令的应答数据包，正常情况飞机返回表示“开始执行”应答数据；但如果飞控正在执行一条之前的指令，则返回“执行失败”的应答数据。飞控开始执行指令之后会尝试切换状态模式，并把执行成功与否的结果保存下来。

第二个阶段是机载设备在发送状态控制指令之后可以开始尝试发送命令码为 0x02 的执行结果查询命令。

|数据类型|偏移（字节）|大小（字节）|说明|
|--------|------------|------------|----|
|请求数据|0|1|指令序列号|
|应答数据|0|1|返回码<ui><li>0x0001：执行失败（指令序列号不是当前执行的指令）</li><li>0x0003：指令正在执行</li><li>0x0004：指令执行失败</li><li>0x0005：指令执行成功</li></ui>


##### 命令码 0x03 姿态控制命令

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td rowspan="5">请求数据</td>
  <td>0</td>
  <td>1</td>
  <td>模式标志位 (详细说明参考飞控控制附加说明章节)</td>
</tr>

<tr>
  <td>1</td>
  <td>4</td>
  <td>roll 方向控制量或 X 轴控制量</td>
</tr>

<tr>
  <td>5</td>
  <td>4</td>
  <td>pitch 方向控制量或 Y 轴控制量</td>
</tr>

<tr>
  <td>9</td>
  <td>4</td>
  <td>yaw 方向控制量（偏航）</td>
</tr>

<tr>
  <td>13</td>
  <td>4</td>
  <td>throttle 方向控制量或 Z 轴控制量</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td></td>
  <td>无应答数据</td>
</tr>

</table>

推荐发送姿态控制命令数据的 C/C++结构体

~~~c
typedef __attribute__((__packed__)) struct { // 1 byte aligned
  unsigned char ctrl_flag;
  float roll_or_x;
  float pitch_or_y;
  float yaw;
  float throttle_or_z;
} control_input;
~~~

**备注：文档中介绍的结构体示例都要求 1 字节对齐。开发者需要根据自身的开发编程环境及编程语言保证结构体的对齐方式为 1 字节对齐。**

根据模式标志位的值，四个输入控制量会被解释成不同含义的输入，有些情况下是 Body 坐标系，有些情况下是 Ground 坐标系。关于坐标系和模式标志位的说明请参考附述章节。

**注意！非常重要：控制模式有进入条件限制：**

- 当且仅当GPS信号正常（health\_flag >=3）时，才可以使用水平**位置**控制（HORI_POS）相关的控制指令
- 当GPS信号正常（health\_flag >=3），或者Gudiance系统正常工作（连接安装正确）时，可以使用水平**速度**控制（HORI_VEL）相关的控制指令

**关于GPS信号健康度的获取，请参考“命令码 0x00 标准数据包”**
**关于位置控制和速度控制相关的指令，请参考附述章节**


#### 命令集 0x02 飞控外发的数据 

##### 命令码 0x00 标准数据包

飞控外发的状态数据包可以通过 DJI N1 PC 调参软件配置。可以配置状态包是否发送及发送的频率。

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td rowspan="13">推送数据</td>
  <td>0</td>
  <td>2</td>
  <td>状态包存在标志位<br>bit 0：时间戳包存在标志<br>bit 1：姿态四元素包存在标志<br>bit 2：Ground 坐标系下的加速度包存在标志<br>bit 3：Ground 坐标系下的速度包存在标志<br>bit 4：Body 坐标系的角速度包存在标志<br>bit 5：GPS 位置、海拔（气压计数值）、相对地面高度、健康度包存在标志<br>bit 6：磁感计数值包存在标志<br>bit 7：遥控器通道值包存在标志<br>bit 8：云台 roll、pitch、yaw 数据包存在标志<br>bit 9：飞行状态包存在标志<br>bit 10：剩余电池百分比包存在标志<br>bit 11：控制设备包存在标志<br>bit [12:15]：保留不用<br><br>标志位为 1 表示标准数据包中存在该状态包</td>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>时间戳</td>
</tr>

<tr>
  <td>6</td>
  <td>16</td>
  <td>姿态四元素</td>
</tr>

<tr>
  <td>22</td>
  <td>12</td>
  <td>Ground 坐标系下的加速度</td>
</tr>

<tr>
  <td>34</td>
  <td>12</td>
  <td>Ground 坐标系下的速度</td>
</tr>

<tr>
  <td>46</td>
  <td>12</td>
  <td>Body 坐标系的角速度</td>
</tr>

<tr>
  <td>58</td>
  <td>24</td>
  <td>GPS 位置, 海拔（气压计数值）, 相对地面高度</td>
</tr>

<tr>
  <td>82</td>
  <td>12</td>
  <td>磁感计数值</td>
</tr>

<tr>
  <td>94</td>
  <td>10</td>
  <td>遥控器通道值</td>
</tr>

<tr>
  <td>104</td>
  <td>12</td>
  <td>云台 roll、pitch、yaw 数据</td>
</tr>

<tr>
  <td>116</td>
  <td>1</td>
  <td>飞行状态</td>
</tr>

<tr>
  <td>117</td>
  <td>1</td>
  <td>剩余电池百分比</td>
</tr>

<tr>
  <td>118</td>
  <td>1</td>
  <td>控制设备</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td></td>
  <td>无应答数据</td>
</tr>
</table>

第一个状态包是时间戳包，之后的状态包偏移字节是不固定的，根据该状态包之前的状态包是否发送而定。

标准数据包中各个状态包的数据段含义如下表所示：

<table>
<tr>
  <td colspan="5" align="middle">标志数据包</td>
</tr>
<tr>
  <td>状态包</td>
  <td>状态包字段</td>
  <td>数据段类型</td>
  <td>说明</td>
  <td>默认频率</td>
</tr>

<tr>
  <td>时间戳</td>
  <td>time</td>
  <td>unsigned int</td>
  <td>时间戳（时间间隔1/600s）</td>
  <td>100Hz</td>
</tr>
<tr>
  <td rowspan="4">姿态四元素</td>
  <td>q0</td>
  <td>float32</td>
  <td rowspan="4">姿态四元数（从 Ground 坐标系转到 Body 坐标系）</td>
  <td rowspan="4">100Hz</td>
</tr>
<tr>
  <td>q1</td>
  <td>float32</td>
</tr>
<tr>
  <td>q2</td>
  <td>float32</td>
</tr>
<tr>
  <td>q3</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="3">Ground 坐标系下的加速度</td>
  <td>agx</td>
  <td>float32</td>
  <td rowspan="3"></td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>agy</td>
  <td>float32</td>
</tr>
<tr>
  <td>agz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="3">Ground 坐标系下的速度</td>
  <td>vgx</td>
  <td>float32</td>
  <td rowspan="3"></td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>vgy</td>
  <td>float32</td>
</tr>
<tr>
  <td>vgz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="3">Body 坐标系下的角速度</td>
  <td>wx</td>
  <td>float32</td>
  <td rowspan="3"></td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>wy</td>
  <td>float32</td>
</tr>
<tr>
  <td>wz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="5">GPS 位置、海拔、相对地面高度、信号健康度</td>
  <td>longti</td>
  <td>double</td>
  <td rowspan="2">GPS 位置</td>
  <td rowspan="5">100Hz</td>
</tr>
<tr>
  <td>lati</td>
  <td>double</td>
</tr>
<tr>
  <td>alti</td>
  <td>float32</td>
  <td>海拔（气压计数值）</td>
</tr>
<tr>
  <td>height</td>
  <td>float32</td>
  <td>相对地面高度（超声波和气压计融合）</td>
</tr>
<tr>
  <td>health_flag</td>
  <td>uint8_t</td>
  <td>GPS 健康度 (0-5, 5 为最好)</td>
</tr>

<tr>
  <td rowspan="3">磁感计</td>
  <td>mx</td>
  <td>float32</td>
  <td rowspan="3">磁感计数值</td>
  <td rowspan="3">0Hz</td>
</tr>
<tr>
  <td>my</td>
  <td>float32</td>
</tr>
<tr>
  <td>mz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="6">遥控器数据</td>
  <td>roll</td>
  <td>int16_t</td>
  <td>遥控通道 roll 数据</td>
  <td rowspan="6">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>int16_t</td>
  <td>遥控通道 pitch 数据</td>
</tr>
<tr>
  <td>yaw</td>
  <td>int16_t</td>
  <td>遥控通道 yaw 数据</td>
</tr>
<tr>
  <td>throttle</td>
  <td>int16_t</td>
  <td>遥控通道 throttle 数据</td>
</tr>
<tr>
  <td>mode</td>
  <td>int16_t</td>
  <td>遥控通道 mode 数据（模式选择开关）</td>
</tr>
<tr>
  <td>gear</td>
  <td>int16_t</td>
  <td>遥控通道 gear 数据（正面的圆形拨杆）</td>
</tr>

<tr>
  <td rowspan="3">云台状态数据</td>
  <td>roll</td>
  <td>float32</td>
  <td>云台 roll 数据</td>
  <td rowspan="3">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>float32</td>
  <td>云台 pitch 数据</td>
</tr>
<tr>
  <td>yaw</td>
  <td>float32</td>
  <td>云台 yaw 数据</td>
</tr>

<tr>
  <td>飞行状态</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>飞行状态</td>
  <td>10Hz</td>
</tr>

<tr>
  <td>电量</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>剩余电量百分比</td>
  <td>1Hz</td>
</tr>

<tr>
  <td>控制设备</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>控制设备<br>0：遥控器<br>1：移动设备<br>2：机载设备</td>
  <td>0Hz</td>
</tr>
</table>

机载设备端可以按照如下 C/C++程序示例接收飞控外发的包含飞机状态的标准数据包

~~~c
typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
}sdk_q_data_t;

typedef struct {
  float x;
  float y;
  float z;
}sdk_common_data_t;

typedef struct {
  double lati;
  double longti;
  float alti;
  float height;
  short health_flag;
}sdk_gps_height_data_t;

typedef struct {
  signed short roll;
  signed short pitch;
  signed short yaw;
  signed short throttle;
  signed short mode;
  signed short gear;
}sdk_rc_data_t;

typedef struct {
  signed short x;
  signed short y;
  signed short z;
}sdk_mag_data_t;

typedef __attribute_((__packed__)) struct { //1 byte aligned
  unsigned int time_stamp;
  sdk_q_data_t          q;
  sdk_common_data_t     a;
  sdk_common_data_t     v;
  sdk_common_data_t     w;
  sdk_gps_height_data   pos;
  sdk_mag_data_t        msg;
  sdk_rc_data_t         rc;
  sdk_common_data_t     gimbal;
  unsigned char         status;
  unsigned char         battery_remaining_capacity;
  unsigned char         ctrl_device;
}sdk_std_data_t;

#define _recv_std_data(_flag, _enable, _data, _buf, _datalen) \
    if(_flag * _enable) { \
      memcpy ((unsigned char*) &(_data), (unsigned char*)(_buf)+(_datalen), sizeof(_data)); \
      _datalen += sizeof(_data); \
    }

static sdk_std_data_t recv_sdk_std_data = {0};

void recv_std_package (unsigned char* pbuf, unsigned int len) {
  unsigned short *valid_flag = (unsigned short*) pbuf;
  unsigned short data_len = 2;
  
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.time_stamp,                 pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0002, recv_sdk_std_data.q,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0004, recv_sdk_std_data.a,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0008, recv_sdk_std_data.v,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0010, recv_sdk_std_data.w,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0020, recv_sdk_std_data.pos,                        pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0040, recv_sdk_std_data.mag,                        pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.rc,                         pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.gimbal,                     pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.statis,                     pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.battery_remaining_capacity, pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.ctrl_device,                pbuf,data_len);
}
~~~

**备注：文档中介绍的结构体示例都要求 1 字节对齐。开发者需要根据自身的开发编程环境及编程语言保证结构体的对齐方式为 1 字节对齐。**

**对数据内容的进一步说明**

_alti_是气压计和IMU融合的结果，单位为气压值；_height_是超声波、气压计和IMU融合的结果，表示相对起飞点的高度，单位是米。如果飞行器上没有超声波传感器（没有安装Guidance），或者有超声波传感器但是相对地面的距离超过3米（距离过远时超声波测量值不稳定），则_height_主要由气压计提供，因此在室内环境中可能会出现超过3米时飞行器因为气压计不稳突然飘动的问题，使用时一定要注意。

因为_height_是相对起飞点的高度，因此如果上电后不起飞，这个数值不会刷新成有意义的值。

_GPS_ 信息中的 _lati_, _longti_ 均为弧度制。

IMU外发的加速度和角速度都是经过滤波算法处理的结果，我们会在未来的版本中加入标志位允许IMU外发传感器的原始数据。

##### 命令码 0x01 控制权归属切换

机载设备的控制权优先级最低，其控制权可能在任何时候被夺去。控制权归属切换数据包会在机载控制权被夺去的时由飞控主动推送。

|数据类型|偏移（字节）|大小（字节）|说明|
|--------|------------|------------|----|
|请求数据|0|1|数据值固定为 0x04|
|应答数据|0|0|无应答数据|

##### 命令码 0x02 透传数据（移动设备至机载设备）

移动设备发送给机载设备的数据包。最大包大小为 100 字节，带宽约 1KB/s。

|数据类型|偏移（字节）|大小（字节）|说明|
|--------|------------|------------|----|
|请求数据|0|1~100|用户自定义数据|
|应答数据|0|0|无应答数据|

