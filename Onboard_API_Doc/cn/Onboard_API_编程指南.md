#Onboard SDK 编程指南

*如发现任何错误，请通过Github issue或开发者论坛或邮件反馈给我们。欢迎提交pull request来帮助我们修正问题，关于文档的修改需要符合[格式标准](https://github.com/dji-sdk/Onboard-SDK/issues/19)*

---

DJI 为开发者提供两种功能完善的飞行控制 API 帮助开发飞行应用：Mobile API 和 Onboard API。Mobile API 是 DJI Mobile SDK 的核心部分，开发者可以基于 iOS/Android 系统上的 SDK
库编写控制飞行器的移动端应用。而 Onboard API 则提供串行接口（UART），允许开发者将 自己的计算设备挂载到飞行器上，通过有线的方式直接控制飞行器。 

本指南中介绍了如何通过程序与 MATRICE 100 交互并发送控制指令。我们推荐开发者先通过快速入门实现我们的示例代码，然后再阅读编程指南。

**本编程指南不包含与Mobile SDK进行数据透传的相关内容**
## 协议说明 

### 协议帧格式

   ```
   |<--------------Protocol Frame Header---------------->|<--Protocol Frame Data-->|<--Protocol Frame Checksum-->|
   |SOF|LEN|VER|SESSION|A|RES0|PADDING|ENC|RES1|SEQ|CRC16|          DATA           |            CRC32            |
   ```

#### 协议帧头段

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
  <td>帧起始标识。固定值 0xAA</td>
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
  <td>ACK</td>
  <td>1</td>
  <td>帧标识<ul>
    <li>0 ： 数据帧</li>
    <li>1 ： 应答帧</li>
    </ul></td>
</tr>

<tr>
  <td>RES0</td>
  <td>2</td>
  <td>保留不用。固定值为 0</td>
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
  <td>帧数据加密类型<ul>
    <li>0 ： 不加密</li>
    <li>1 ： AES 加密</li>
    </ul></td>
</tr>

<tr>
  <td>RES1</td>
  <td>5</td>
  <td>24</td>
  <td>保留。固定值为 0</td>
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
  <td>帧数据段。长度不定，最大长度为 1007bytes</td>
</tr>

<tr>
  <td>CRC32</td>
  <td>---</td>
  <td>32</td>
  <td>帧 CRC32 校验值</td>
</tr>
</table>



#### 协议数据段

协议数据段根据数据传输方向和传输内容分为三类

|数据包类型|传输方向|传输内容|
|------------|:----------:|---------------|
|命令数据包|机载设备==>飞控|飞行器控制指令|
|应答数据包|飞控==>机载设备|控制指令的执行结果
|推送数据包|飞控==>机载设备|飞控的状态信息和传感器数据

##### 命令数据包 

```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|字段|字节索引|大小（单位 byte）|说明|
|----|--------|-----------------|----|
|COMMAND SET|0|1|命令集|
|COMMAND ID|1|1|命令码|
|COMMAND DATA|2|与命令码有关|命令数据|

##### 应答数据包
*应答数据包的帧头部分帧标识（ACK）为1*

```
|<-Protocol Frame Data->|
|COMMAND RETURN|ACK DATA|
```


|字段|字节索引|大小（单位 byte）|说明|
|----|--------|-----------------|----|
|COMMAND RETURN|0|2|返回码|
|ACK DATA|2|与命令有关|返回数据|


##### 推送数据包

```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|字段|字节索引|大小（单位 byte）|说明|
|----|--------|-----------------|----|
|COMMAND SET|0|1|命令集|
|COMMAND ID|1|1|命令码|
|COMMAND DATA|2|与命令码有关|飞行器状态*|

*\*飞行器状态包含了飞行器姿态、加速度、GPS、高度等信息，具体包含内容通过调参软件配置*

---

### 协议通信机制


#### 会话机制

协议设计使用了会话机制，以保证命令数据和应答数据不会因为丢包而出现通信双方异常。通信双方在向对方发起通信会话时，可以根据需要通过设置协议的 SESSION 字段来选择会话方式。协议中设计了三种会话方式。

|会话方式|SESSION|描述|
|--------|-------|----|
|方式1|0|发送端不需要接收端应答|
|方式2|1|发送端需要接收端应答数据，但是可以容忍应答数据丢包|
|方式3|2-31|发送端需要正确收到接收端的应答包。*|

*\*发送端使用这些SESSION 发送命令数据包时，应答包中包含该命令数据包中的 帧序列号(SEQ) 和 通信过程中的会话 ID (SESSION)。如果通信过程中，发送端没有正确收到应答包，可以重新发送命令数据包。*


*备注：由于会话方式 3 是一种可靠会话方式，开发者在协议链路层实现中应考虑数据丢包后的重发机制，在设计链路层发送接口时应提供超时时间、重发次数等参数。*

---

## 命令集与命令码

### 命令集

**Onboard API 命令分为三大命令集**  

|命令集|命令集代码|说明|
|------|---------|----|
|激活验证类|0x00|机载设备查询飞控激活状态完成激活的相关命令|
|控制命令类|0x01|机载设备完成对飞行器及云台等设备控制的相关命令|
|推送数据类|0x02|由飞控主动向机载设备的数据，如传感器数据、云台数据等

### 命令码
命令集包含一系列命令码，根据具体的命令码实现相应的功能。  

*命令需要在相应的权限级别下才能够被执行。当机载设备发出的命令所需的权限级别高于飞控所处权限级别时，该命令将不会被执行。较高的权限级别下可以执行低级别权限命令*

|权限级别|权限描述|
|:-----:|--------|
|0|API 激活命令|
|1|相机和云台的控制命令|
|2|飞行控制命令|

*权限级别可通过激活命令改变。飞行器激活前默认权限级别为0*


**Onboard API 功能索引**
<table>
<tr>
  <th>命令集</th>
  <th>命令码</th>
  <th>功能</th>
  <th>所需权限级别</th>
</tr>
  <td rowspan="2">0x00<br>激活验证类 </td>
  <td>0x00</td>
  <td>获取 API 版本</td>
  <td>0</td>
</tr>

</tr>
  <td>0x01</td>
  <td>激活</td>
  <td>0</td>
</tr>

<tr>
  <td rowspan="6">0x01<br>控制命令类 </td>
  <td>0x00</td>
  <td>请求获得控制权</td>
  <td>2</td>
</tr>

<tr>
  <td>0x01</td>
  <td>切换飞行状态</td>
  <td>2</td>
</tr>
<tr>
  <td>0x02</td>
  <td>查询飞行状态切换结果</td>
  <td>2</td>
</tr>

<tr>
  <td>0x03</td>
  <td>姿态控制</td>
  <td>2</td>
</tr>

<tr>
  <td>0x0A</td>
  <td>云台角速度控制</td>
  <td>1</td>
</tr>

<tr>
  <td>0x0B</td>
  <td>云台角度控制</td>
  <td>1</td>
</tr>

<tr>
  <td rowspan="2">0x02<br>推送数据类</td>
  <td>0x00</td>
  <td>推送数据</td>
  <td>0</td>
</tr>

<tr>
  <td>0x01</td>
  <td>失去控制权</td>
  <td>0</td>
</tr>
</table>


## 命令数据说明

### 命令集 0x00 激活验证类 

#### 命令码 0x00 获取 API 版本

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr> 
  <td>请求数据</td>
  <td>0</td>
  <td>1</td>
  <td>保留</td>
</tr>

<tr>
  <td rowspan="3">应答数据</td>
  <td>0</td>
  <td>2</td>
  <td>返回码<ul>
    <li>0x0000：激活成功</li>
    <li>0xFF00：命令不支持</li>
    <li>0xFF01：机载设备无授权</li>
    <li>0xFF02：机载设备权限不足</li></ul>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>API版本号校验值</td>
</tr>

<tr>
  <td>6</td>
  <td>32</td>
  <td>API版本号</td>
</tr>
</table>


#### 命令码 0x01 激活

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
  <td>app_id, 服务器注册时候生成的内容</td>
</tr>

<tr>
  <td>4</td>
  <td>4</td>
  <td>api_level，API 权限级别</td>
</tr>

<tr>
  <td>8</td>
  <td>4</td>
  <td>固定值，0x02030A00</td>
</tr>

<tr>
  <td>12</td>
  <td>32</td>
  <td>保留</td>
</tr>

<tr>
  <td>应答数据</td>
  <td>0</td>
  <td>2</td>
  <td>返回码：<ul>
    <li>0x0000：成功</li>
    <li>0x0001：参数非法</li>
    <li>0x0002：数据包加密，未能正确识别</li>
    <li>0x0003：没有激活过的设备，尝试激活</li>
    <li>0x0004：DJI GO 没有响应 </li>
    <li>0x0005：DJI GO 没有联网</li>
    <li>0x0006：服务器拒绝，激活失败</li>
    <li>0x0007：权限级别不够</li>
    <li>0x0008：SDK版本错误</li>
    </ul></td>
</tr>

</table>

### 命令集 0x01 控制命令类 

#### 命令码 0x00 请求获得控制权

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td >请求数据</td>
  <td>0</td>
  <td>1</td>
  <td>请求命令<ul>
    <li>0x01 ： 请求获得控制权</li>
    <li>0x00 ： 请求释放控制权</li>
    </ul></td>
</tr>

<tr>
 <td >应答数据</td>
  <td>0</td>
  <td>2</td>
  <td>返回码 <ul>
    <li>0x0001：成功释放控制权</li>
    <li>0x0002：成功获得控制权</li>
    <li>0x0003：正在获取控制权</li>
    </ul></td>
</tr>

</table>

飞行器可以接受三种设备的控制输入：遥控器、移动设备、机载设备而。三种设备的控制输入的优先级最大是遥控器，其次是移动设备，优先级最低是机载设备。假设请求获得控制权命令的回调函数为：

#### 命令码 0x01 切换飞行状态

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
  <td>控制命令<ul>
    <li>0x01 ： 请求进入自动返航</li>
    <li>0x04 ： 请求自动起飞</li>
    <li>0x06 ： 请求自动降落</li>
    </ul></td>
</tr>

<tr>
  <td>应答数据</td>
  <td>0</td>
  <td>2</td>
  <td>返回码<ul>
    <li>0x0001：执行失败</li>
    <li>0x0002：开始执行</li>
    </ul></td>
</tr>

</table>

#### 命令码 0x02 查询飞行状态切换结果
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td >请求数据</td>
  <td>0</td>
  <td>2</td>
  <td>指令序列号</td>
</tr>


<tr>
  <td>应答数据</td>
  <td>0</td>
  <td>2</td>
  <td>返回码<ul>
    <li>0x0001：执行失败（指令序列号不是当前执行的指令）</li>
    <li>0x0003：指令正在执行</li>
    <li>0x0004：指令执行失败</li>
    <li>0x0005：指令执行成功</li>
    </ul></td>
</tr>

</table>

#### 命令码 0x03 姿态控制\*
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
  <td>模式标志字节</td>
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
  <td>应答数据</td>
  <td>---</td>
  <td>---</td>
  <td>无应答数据</td>
</tr>

</table>

*\*有关姿态控制的具体内容请参阅[飞行控制附加说明][0]X*  

**注意！非常重要：控制模式有进入条件限制：**

- 当且仅当GPS信号正常（health\_flag >=3）时，才可以使用水平**位置**控制（HOR_POS）相关的控制指令
- 当GPS信号正常（health\_flag >=3），或者Gudiance系统正常工作（连接安装正确）时，可以使用水平**速度**控制（HORI_VEL）相关的控制指令

**关于GPS信号健康度的获取，请参考“命令码 0x00 标准数据包”**
**关于位置控制和速度控制相关的指令，请参考附述章节**


#### 命令码 0x0A 云台角速度控制
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
  <td>2</td>
  <td>Yaw轴角速度</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>Roll轴角速度</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>Pitch轴角速度</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>固定值，0x80</td>
</tr>

<tr>
  <td>应答数据</td>
  <td>---</td>
  <td>---</td>
  <td>无应答数据</td>
</tr>

</table>


<table>
<tr>
  <th>数据名称</th>
  <th>数据类型</th>
  <th>说明</th>
</tr>

<tr>
  <td>Yaw轴角速度</td>
  <td>int16_t</td>
  <td>单位0.1度/s，输入范围（[-1800,+1800]）</td>
</tr>

<tr>
  <td>Roll轴角速度</td>
  <td>int16_t</td>
  <td>单位0.1度/s，输入范围[-1800,+1800]</td>
</tr>

<tr>
  <td>Pitch轴角速度</td>
  <td>int16_t</td>
  <td>单位0.1度/s，输入范围[-1800,+1800]</td>
</tr>
</table>

#### 命令码 0x0B 云台角度控制
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
  <td>2</td>
  <td>Yaw轴角度</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>Roll轴角度</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>Pitch轴角度</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>属性控制字节<ul>
    <li>bit 0：控制模式选择位</li>
        <ul>0 ： 增量控制，角度基准为当前云台所处位置</ul>
        <ul>1 ： 绝对控制，角度基准为东北地坐标系</ul>
    <li>bit 1：Yaw轴命令控制失效位 
        <ul>0 ： 云台Yaw角度运动到命令位置 </ul>
        <ul>1 ： 云台Yaw将维持上一时刻状态 </ul>
    <li>bit 2：Roll轴命令控制失效位，同bit[1]描述</li>
    <li>bit 3：Pitch轴命令控制失效位，同bit[1]描述</li>
    <li>bit [4:7]：保留，必须为0</li>
    </ul></td>

<tr>
  <td>7</td>
  <td>1</td>
  <td>命令完成时间</td>
</tr>
</tr>

<tr>
  <td>应答数据</td>
  <td>---</td>
  <td>---</td>
  <td>无应答数据</td>
</tr>
</table>

<table>
<tr>
  <th>数据名称</th>
  <th>数据类型</th>
  <th>说明</th>
</tr>

<tr>
  <td>Yaw轴角度</td>
  <td>int16_t</td>
  <td>单位0.1度，输入范围 [-3200~+3200]</td>
</tr>

<tr>
  <td>Roll轴角度</td>
  <td>int16_t</td>
  <td>单位0.1度，输入范围 [-350~+350]</td>
</tr>

<tr>
  <td>Pitch轴角度</td>
  <td>int16_t</td>
  <td>单位0.1度，输入范围 [-900~+300]</td>
</tr>

<tr>
  <td>命令完成时间</td>
  <td>uint8_t</td>
  <td>单位0.1s，例如20代表云台在2s内匀速转动至命令位置<br>建议用户控制速度不超过400度/秒</td>
</tr>
</table>

### 命令集 0x02 推送数据类

#### 命令码 0x00 推送数据

飞控外发的状态数据包可以通过 DJI N1 PC 调参软件配置。可以配置状态包是否发送及发送的频率。  

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）*</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td rowspan="13">推送数据</td>
  <td>0</td>
  <td>2</td>
  <td>状态包存在标志位，标志位为 1 表示标准数据包中存在该状态包<ul>
    <li>bit 0：时间戳包存在标</li>
    <li>bit 1：姿态四元素包存在标志</li>
    <li>bit 2：Ground 坐标系下的加速度包存在标志</li>
    <li>bit 3：Ground 坐标系下的速度包存在标志</li>
    <li>bit 4：Body 坐标系的角速度包存在标志</li>
    <li>bit 5：GPS 位置、海拔（气压计数值）、相对地面高度、健康度包存在标志</li>
    <li>bit 6：磁感计数值包存在标志</li>
    <li>bit 7：遥控器通道值包存在标志</li>
    <li>bit 8：云台 roll、pitch、yaw 数据包存在标志</li>
    <li>bit 9：飞行状态包存在标志</li>
    <li>bit 10：剩余电池百分比包存在标志</li>
    <li>bit 11：控制设备包存在标志</li>
    <li>bit [12:15]：保留不用</li>
    </td>></ul>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>时间戳</td>
</tr>

<tr>
  <td>6</td>
  <td>16</td>
  <td>姿态四元数</td>
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
  <td>云台姿态</td>
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
</table>

*\*偏移（字节）：表格中偏移（字节）为推送数据中存在所有状态包的情况。*

 实际数据在推送数据中的偏移需要根据状态包存在标志位确定存在的状态包，然后根据各状态包大小计算出状态包的实际偏移大小。
 
**数据包中各个状态包的数据段含义**

<table>
<tr>
  <th>状态包</th>
  <th>状态包字段</th>
  <th>数据段类型</th>
  <th>说明</th>
  <th>单位</th>
  <th>默认频率</th>
</tr>

<tr>
  <td>时间戳</td>
  <td>time</td>
  <td>unsigned int</td>
  <td>时间戳</td>
  <td>时间间隔1/600s</td>>
  <td>100Hz</td>
</tr>
<tr>
  <td rowspan="4">姿态四元数</td>
  <td>q0</td>
  <td>float32</td>
  <td rowspan="4">姿态四元数<br>从 Ground 坐标系转到 Body 坐标系变换*</td>
  <td rowspan="4">---</td>
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
  <td rowspan="3">加速度</td>
  <td>agx</td>
  <td>float32</td>
  <td rowspan="3">飞行器在 Ground 坐标系下加速度</td>
  <td rowspan="3">m/s<sup>2</sup> </td>
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
  <td rowspan="3">速度</td>
  <td>vgx</td>
  <td>float32</td>
  <td rowspan="3">飞行器在 Ground 坐标系下速度</td>
  <td rowspan="3">m/s</td>
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
  <td rowspan="3">角速度</td>
  <td>wx</td>
  <td>float32</td>
  <td rowspan="3">飞行器在 Body 坐标系下角速度 </td>
  <td rowspan="3">度/s</td>
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
  <td rowspan="5">GPS及高度</td>
  <td>longti</td>
  <td>double</td>
  <td rowspan="2">GPS 位置</td>
  <td rowspan="2">rad</td>
  <td rowspan="5">100Hz</td>
</tr>
<tr>
  <td>lati</td>
  <td>double</td>
</tr>
<tr>
  <td>alti</td>
  <td>float32</td>
  <td>海拔</td>
  <td>---</td>
</tr>
<tr>
  <td>height</td>
  <td>float32</td>
  <td>相对地面高度**</td>
  <td>m</td>
</tr>
<tr>
  <td>health_flag</td>
  <td>uint8_t</td>
  <td>GPS 健康度 </td>
  <td>0-5, 5 为最好</td>
</tr>

<tr>
  <td rowspan="3">磁感计</td>
  <td>mx</td>
  <td>int16_t</td>
  <td rowspan="3">磁感计数值</td>
  <td rowspan="3">磁感计数值</td>
  <td rowspan="3">0Hz</td>
</tr>
<tr>
  <td>my</td>
  <td>int16_t</td>
</tr>
<tr>
  <td>mz</td>
  <td>int16_t</td>
</tr>

<tr>
  <td rowspan="6">遥控器</td>
  <td>roll</td>
  <td>int16_t</td>
  <td>遥控通道 roll 数据</td>
  <td rowspan="6">---</td>
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
  <td>遥控通道 gear 数据（返航键外圈拨杆）</td>
</tr>

<tr>
  <td rowspan="3">云台姿态</td>
  <td>roll</td>
  <td>float32</td>
  <td rowspan="3">云台在Ground 坐标系下姿态</td>
  <td rowspan="3">度</td>
  <td rowspan="3">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>float32</td>
</tr>
<tr>
  <td>yaw</td>
  <td>float32</td>
</tr>

<tr>
  <td>飞行状态*</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>飞行状态</td>
  <td>---</td>
  <td>10Hz</td>
</tr>

<tr>
  <td>电量</td>
  <td>battery</td>
  <td>uint8_t</td>
  <td>剩余电量百分比</td>
  <td>%</td>
  <td>1Hz</td>
</tr>

<tr>
  <td>控制信号源</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>控制设备<ul>
     <li>0x00 ： 遥控器</li>
     <li>0x01 ： 移动设备</li>
     <li>0x02 ： 机载设备</li>
     </ul></td>
  <td>---</td>
  <td>0Hz</td>
</tr>
</table>

*\*Ground 坐标系、Body 坐标系、遥控器及飞行状态相关详细说明请参阅[飞行控制附加说明][0]*  
*\*\*相对地面高度是超声波、气压计和IMU融合的结果。如果飞行器上没有安装Guidance，或者安装Guidance但是相对地面的距离超过3米，相对地面高度则由气压计气压计计算得出。室内无法准确获取气压值，此数据将不可靠。*


#### 命令码 0x01 失去控制权
失去控制权数据包会在机载控制权被夺去的时由飞控主动推送。  
机载设备的控制权优先级最低，其控制权可能在任何时候被夺去。

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td >推送数据</td>
  <td>0</td>
  <td>1</td>
  <td>固定值，0x04</td>
</tr>
</table>


[0]: 飞行控制附加说明.md