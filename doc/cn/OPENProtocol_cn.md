#### 本文档已停止维护, 请移步到DJI开发者官网的 <https://developer.dji.com/onboard-sdk/documentation/introduction/index.html> 查看最新版本. 

#Onboard SDK 开放协议说明
---

## 协议帧
协议帧是本开放协议中信息传输的最小单位，由帧头，帧数据段及帧尾组成。以下为帧格式:

   ```
   |<-----------------------帧头段------------------------->|<--帧数据段-->|<--帧尾段-->|
   |SOF|LEN|VER|SESSION|ACK|RES0|PADDING|ENC|RES1|SEQ|CRC16|     DATA    |    CRC32   |
   ```

### 帧结构

<table>
<tr>
  <th>字段</th>
  <th>索引（byte）</th>
  <th>大小（bit）</th>
  <th>说明</th>
</tr>

<tr>
  <td>SOF</td>
  <td>0</td>
  <td>8</td>
  <td>帧起始标识，固定为0xAA</td>
</tr>

<tr>
  <td>LEN</td>
  <td rowspan="2">1</td>
  <td>10</td>
  <td>帧长度标识</td>
</tr>

<tr>
  <td>VER</td>
  <td>6</td>
  <td>帧头格式版本，固定为0</td>
</tr>

<tr>
  <td>SESSION</td>
  <td rowspan="3">3</td>
  <td>5</td>
  <td>会话ID</td>
</tr>

<tr>
  <td>ACK</td>
  <td>1</td>
  <td>帧标识<ul>
    <li>0:命令帧</li>
    <li>1:应答帧</li>
    </ul></td>
</tr>

<tr>
  <td>RES0</td>
  <td>2</td>
  <td>保留。固定值为0</td>
</tr>

<tr>
  <td>PADDING</td>
  <td rowspan="2">4</td>
  <td>5</td>
  <td>加密 帧数据段 时所需的附加数据长度</td>
</tr>

<tr>
  <td>ENC</td>
  <td>3</td>
  <td>加密类型<ul>
    <li>0:不加密</li>
    <li>1:AES加密</li>
    </ul></td>
</tr>

<tr>
  <td>RES1</td>
  <td>5</td>
  <td>24</td>
  <td>保留。固定值为0</td>
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
  <td>长度不定</td>
  <td>帧数据段</td>
</tr>

<tr>
  <td>CRC32</td>
  <td>大小不定</td>
  <td>32</td>
  <td>整个帧的 CRC32 校验值</td>
</tr>

</table>
>备注:如果协议帧需要AES加密，应先对数据段进行加密，该过程可能会产生附加的数据长度，需要更新帧头中*PADDING*字段及*LEN*字段。再对帧头部分进行CRC16校验，获得校验值后再对整个协议帧进行CRC32校验。


### 帧类型

帧分为两类

|帧类型|数据段类型|传输方向|传输内容|
|------|------------|:----------:|---------------|
|命令帧|命令数据段|机载设备<=>飞控|飞行器控制指令|
|应答帧|应答数据段|飞控<=>机载设备|控制指令的执行结果|

#### 命令帧数据段

```
|<-------帧数据段------->|
|CMD SET|CMD ID|CMD VALUE|
```

|字段|索引（byte）|大小（byte）|说明|
|----|--------|-----------------|----|
|CMD SET|0|1|命令集|
|CMD ID|1|1|命令码|
|CMD VALUE|2|与命令码有关|命令值|

#### 应答帧数据段

```
|<--帧数据段-->|
|   ACK VALUE  |
```

|字段|索引（byte）|大小（byte）|说明|
|----|--------|-----------------|----|
|ACK VALUE|0|大小可变|应答值|

当接收到应答帧时，应答帧帧头部分中包含相应命令帧的帧序列号(SEQ)，开发者可通过此信息匹配应答帧与命令帧。


---

## 协议通信机制


### 会话机制

协议设计了会话机制，以保证命令数据和应答数据不会因为丢包而出现通信双方异常。通信双方在向对方发起通信会话时，可以根据需要通过设置在帧头部分的SESSION字段来选择会话类型。协议中设计了三种会话类型。



|会话类型*|SESSION|描述|
|--------|-------|----|
|类型0|0|发送端不需要接收端应答|
|类型1|1|发送端需要接收端应答值，但可容忍应答值丢包|
|类型2|2-31|发送端需要正确收到接收端的应答包*|

>备注:   
>**会话类型1及类型2仅适用于具有应答值的命令。*  
>***发送端使用这些 SESSION 发送命令数据包时，应答帧中包含该命令帧中的帧序列号 (SEQ)和通信过程中的会话ID (SESSION)。如果在通信过程中，发送端没有正确收到应答包，可以重新发送包含相同SESSION和SEQ的命令帧。由于会话方式3是一种可靠会话方式，开发者在协议实现中应考虑并实现数据丢包发生时的重发机制等。*

### 加密机制（可选）  
考虑到机载设备与飞控之间的串口通讯可能采用不安全的方式，例如各类无线透传模块等。DJI为通讯提供了加密机制。加密仅对帧数据段进行AES加密，开发者可通过注册APP ID时获得的KEY，使用DJI提供的加密函数进行数据加密。由于加密是16字节对齐，加密后帧数据段长度会发生变化，因此需要在帧头”PADDING“字段写加密时产生的附加数据长度，同时需要重新计算帧的长度标识”LEN“，并将加密类型”ENC“段置1。具体的加密算法及实现请参考`DJI_Pro_Codec.cpp`文件中的`sdk_encrypt_interface`函数。

如果开发者采用飞机直接搭载机载设备，通过有线串口的方式连接机载设备与飞控。那么开发者也可以不采用加密方式通讯，此时仅需将加密类型”ENC“段置0。

---

##  命令集与命令码

### 命令集与命令码
命令集包含一系列命令码，根据具体的命令码实现相应的功能。  


### 功能索引
<table>
<tr>
  <th>命令集</th>
  <th>命令码</th>
  <th>功能</th>
</tr>
  <td rowspan="4">0x00<br>初始化设置类</td>
  <td>0x00</td>
  <td>获取通信协议版本</td>
</tr>

</tr>
  <td>0x01</td>
  <td>激活</td>
</tr>

</tr>
  <td>0x10</td>
  <td>设置推送数据频率</td>
</tr>

</tr>
  <td>0xFE</td>
  <td>数据透传:从Onboard SDK至Mobile SDK</td>
</tr>

<tr>
  <td rowspan="10">0x01<br>控制命令类 </td>
  <td>0x00</td>
  <td>请求获取/释放控制权</td>
</tr>

<tr>
  <td>0x01</td>
  <td>切换飞行状态</td>
</tr>
<tr>
  <td>0x02</td>
  <td>查询飞行状态切换结果</td>
</tr>

<tr>
  <td>0x03</td>
  <td>姿态控制</td>
</tr>
<tr>
  <td>0x05</td>
  <td>解锁/锁定电机</td>
</tr>
<tr>
  <td>0x1A</td>
  <td>云台角速度控制</td>
</tr>

<tr>
  <td>0x1B</td>
  <td>云台角度控制</td>
</tr>

<tr>
  <td>0x20</td>
  <td>相机拍照</td>
</tr>

<tr>
  <td>0x21</td>
  <td>相机开始录像</td>
</tr>

<tr>
  <td>0x22</td>
  <td>相机停止录像</td>
</tr>
<tr>
  <td rowspan="5">0x02<br>推送数据类</td>
  <td>0x00</td>
  <td>飞行数据</td>
</tr>

<tr>
  <td>0x01</td>
  <td>失去控制权</td>
</tr>

<tr>
  <td>0x02</td>
  <td>数据透传:从Mobile SDK至Onboard SDK</td>
</tr>
<tr>
  <td>0x03</td>
  <td>地面站状态</td>
</tr>
<tr>
  <td>0x04</td>
  <td>航点事件</td>
</tr>
<tr>
  <td rowspan="8">0x03<br>地面站功能类<br>航点部分</td>
  <td>0x10</td>
  <td>上传航点任务信息</td>
</tr>
<tr>
  <td>0x11</td>
  <td>上传航点</td>
</tr>
<tr>
  <td>0x12</td>
  <td>启动/停止航点功能</td>
</tr>
<tr>
  <td>0x13</td>
  <td>暂停/恢复航点功能</td>
</tr>
<tr>
  <td>0x14</td>
  <td>下载航点任务信息</td>
</tr>
<tr>
  <td>0x15</td>
  <td>下载航点信息</td>
</tr>
<tr>
  <td>0x16</td>
  <td>设置巡航速度</td>
</tr>
<tr>
  <td>0x17</td>
  <td>读取巡航速度</td>
</tr>
<tr>
  <td rowspan="8">0x03<br>地面站功能类<br>热点部分</td>
  <td>0x20</td>
  <td>启动热点功能</td>
</tr>
<tr>
  <td>0x21</td>
  <td>停止热点功能</td>
</tr>
<tr>
  <td>0x22</td>
  <td>暂停/恢复热点功能</td>
</tr>
<tr>
  <td>0x23</td>
  <td>设置巡航速度</td>
</tr>
<tr>
  <td>0x24</td>
  <td>设置半径</td>
</tr>
<tr>
  <td>0x25</td>
  <td>重置方向</td>
</tr>
<tr>
  <td>0x26</td>
  <td>下载热点任务信息</td>
</tr>
<tr>
  <td>0x27</td>
  <td>启动自动半径模式</td>
</tr>
<tr>
  <td rowspan="4">0x03<br>地面站功能类<br>跟随部分</td>
  <td>0x30</td>
  <td>启动跟随功能</td>
</tr>
<tr>
  <td>0x31</td>
  <td>停止跟随功能</td>
</tr>
<tr>
  <td>0x32</td>
  <td>暂停/恢复跟随功能</td>
</tr>
<tr>
  <td>0x33</td>
  <td>设置跟踪目标点位置</td>
</tr>
<tr>
  <td>0x04<br>同步信号类</td>
  <td>0x00</td>
  <td>设置同步信号输出频率</td>

</tr>
<tr>
  <td rowspan="2">0x05<br>虚拟遥控类</td>
  <td>0x00</td>
  <td>虚拟遥控控制请求</td>
</tr>
<tr>
  <td>0x01</td>
  <td>虚拟遥控数据</td>
</tr>
</table>


## 命令数据说明

### 命令集 0x00 初始化设置类 

#### 命令码 0x00 获取通信协议版本

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr> 
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>任意值</td>
</tr>

<tr>
  <td rowspan="3">应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码<ul>
    <li>0x0000:机载设备已激活</li>
    <li>0xFF01:机载设备未激活</li>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>通信协议版本号校验值</td>
</tr>

<tr>
  <td>6</td>
  <td>32</td>
  <td>通信协议版本号</td>
</tr>
</table>


#### 命令码 0x01 激活

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="3">命令值</td>
  <td>0</td>
  <td>4</td>
  <td>app_id, 应用唯一标识</td>
</tr>


<tr>
  <td>8</td>
  <td>4</td>
  <td>固定值:<ul>
   M100: 0x03010A00<br>A3: 0x03016400
  </ul></td>
</tr>

<tr>
  <td>12</td>
  <td>32</td>
  <td>固定字符串，"12345678901234567890123456789012"</td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码:<ul>
    <li>0x0000:成功</li>
    <li>0x0001:参数非法</li>
    <li>0x0002:数据包加密，未能正确识别</li>
    <li>0x0003:新的APP，请连接DJI GO</li>
    <li>0x0004:DJI GO 没有响应 </li>
    <li>0x0005:DJI GO 没有联网</li>
    <li>0x0006:服务器拒绝</li>
    <li>0x0007:权限级别不够</li>
    <li>0x0008:SDK版本错误</li>
    </ul></td>
</tr>

</table>

#### 命令码 0x10 设置推送数据频率

M100: 
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="13">命令值</td>
  <td>0</td>
  <td>1</td>
  <td>时间戳</td>
</tr>
<tr>
  <td>1</td>
  <td>1</td>
  <td>姿态四元数</td>
</tr>
<tr>
  <td>2</td>
  <td>1</td>
  <td>加速度</td>
</tr>
<tr>
  <td>3</td>
  <td>1</td>
  <td>速度</td>
</tr>
<tr>
  <td>4</td>
  <td>1</td>
  <td>角速度</td>
</tr>
<tr>
  <td>5</td>
  <td>1</td>
  <td>GPS 位置, 海拔高度, 相对地面高度</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>磁感计数值</td>
</tr>
<tr>
  <td>7</td>
  <td>1</td>
  <td>遥控器通道值</td>
</tr>
<tr>
  <td>8</td>
  <td>1</td>
  <td>云台姿态</td>
</tr>
<tr>
  <td>9</td>
  <td>1</td>
  <td>飞行状态</td>
</tr>
<tr>
  <td>10</td>
  <td>1</td>
  <td>剩余电池百分比</td>
</tr>
<tr>
  <td>11</td>
  <td>1</td>
  <td>控制设备</td>
</tr>
<tr>
  <td>12</td>
  <td>4</td>
  <td>保留</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码:<ul>
    <li>0x0000:成功</li>
    <li>0x0001:参数非法</li>
    </ul></td>
</tr>

</table>

A3/M600:

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="15">命令值</td>
  <td>0</td>
  <td>1</td>
  <td>时间戳</td>
</tr>
<tr>
  <td>1</td>
  <td>1</td>
  <td>姿态四元数</td>
</tr>
<tr>
  <td>2</td>
  <td>1</td>
  <td>加速度</td>
</tr>
<tr>
  <td>3</td>
  <td>1</td>
  <td>速度</td>
</tr>
<tr>
  <td>4</td>
  <td>1</td>
  <td>角速度</td>
</tr>
<tr>
  <td>5</td>
  <td>1</td>
  <td>GPS 位置, 海拔高度, 相对地面高度</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>GPS 详细信息</td>
</tr>
<tr>
  <td>7</td>
  <td>1</td>
  <td>RTK 详细信息</td>
</tr>
<tr>
  <td>8</td>
  <td>1</td>
  <td>磁感计数值</td>
</tr>
<tr>
  <td>9</td>
  <td>1</td>
  <td>遥控器通道值</td>
</tr>
<tr>
  <td>10</td>
  <td>1</td>
  <td>云台姿态</td>
</tr>
<tr>
  <td>11</td>
  <td>1</td>
  <td>飞行状态</td>
</tr>
<tr>
  <td>12</td>
  <td>1</td>
  <td>剩余电池百分比</td>
</tr>
<tr>
  <td>13</td>
  <td>1</td>
  <td>控制设备</td>
</tr>
<tr>
  <td>14</td>
  <td>2</td>
  <td>保留</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码:<ul>
    <li>0x0000:成功</li>
    <li>0x0001:参数非法</li>
    </ul></td>
</tr>

</table>

|设置值|推送频率|
|-----|------|
|0|0Hz|
|1|1Hz|
|2|10Hz|
|3|50Hz|
|4|100Hz|
|5|保持不变|

### 命令集 0x01 控制命令类 

#### 命令码 0x00 请求获取/释放控制权

请确认已满足以下条件

* 在PC assistant调参软件中，“启用API控制”勾选框已被勾选
* 遥控器的模式选择开关置于F档 

由于飞控中的保护机制，所有的获取控制权与释放控制权指令需要发送**两次**才会生效，其中第一次会返回失败。故官方库文件中控制权获取和释放控制权的函数均发送了两次协议请求。

**注意！在3.0及之后的固件中，若上电时遥控器模式开关已经处于F档，则默认进入F档逻辑。此处不同于2.3版本固件下的设定，无需将开关拨出F当后再次拨入。请从2.3版本升级的开发者一定注意！**

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 : 命令获得控制权<br>
    0x00 : 命令释放控制权
  </td>
</tr>

<tr>
 <td >应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码 <ul>
    <li>0x0000: 遥控器未置于F档</li>
    <li>0x0001: 释放控制权成功</li>
    <li>0x0002: 获得控制权成功</li>
    <li>0x0003: 获取控制权失败</li>
    <li>0x0004：释放控制权失败</li>
    <li>0x00C9：IOC模式开启</li>
    </ul></td>
</tr>

</table>


#### 命令码 0x01 切换飞行状态

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="2">命令值</td>
  <td>0</td>
  <td>1</td>
  <td>指令序列号*</td>
</tr>

<tr>
  <td>1</td>
  <td>1</td>
  <td>
    0x01 : 自动返航<br>
    0x04 : 自动起飞<br>
    0x06 : 自动降落<br>
   </td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码<ul>
    <li>0x0001:执行失败</li>
    <li>0x0002:开始执行</li>
    </ul></td>
</tr>

</table>

>备注: 不能在电机启动的状态下自动起飞

#### 命令码 0x02 查询飞行状态切换结果
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>指令序列号</td>
</tr>


<tr>
  <td>应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码<ul>
    <li>0x0001:指令序列号错误</li>
    <li>0x0003:指令正在执行</li>
    <li>0x0004:指令执行失败</li>
    <li>0x0005:指令执行成功</li>
    </ul></td>
</tr>

</table>

#### 命令码 0x03 姿态控制

关于模式标志字节说明详情请参阅[附录](Appendix_cn.md#模式标志字节说明)模式标志字节说明部分。

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="5">命令值</td>
  <td>0</td>
  <td>1</td>
  <td>---</td>
  <td>模式标志字节</td>
</tr>

<tr>
  <td>1</td>
  <td>4</td>
  <td>float32</td>
  <td>Roll 轴或 X 轴控制量</td>
</tr>

<tr>
  <td>5</td>
  <td>4</td>
  <td>float32</td>
  <td>Pitch 轴或 Y 轴控制量</td>
</tr>

<tr>
  <td>9</td>
  <td>4</td>
  <td>float32</td>
  <td>Throttle 或 Z 轴控制量</td>
</tr>

<tr>
  <td>13</td>
  <td>4</td>
  <td>float32</td>
  <td>Yaw 轴控制量</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>

</table>


#### 命令码 0x05 解锁/锁定电机
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 : 解锁电机<br>
    0x00 : 锁定电机
  </td>
</tr>

<tr>
 <td >应答值</td>
  <td>0</td>
  <td>2</td>
  <td>返回码 <ul>
    <li>0x0000:成功解锁/锁定电机</li>
    <li>0x0001:未获取控制权</li>
    <li>0x0002:电机已经解锁/锁定</li>
    <li>0x0003:在空中无法锁定</li>
    </ul></td>
</tr>

</table>
#### 命令码 0x1A 云台角速度控制
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="4">命令值</td>
  <td>0</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Yaw轴角速度<br>单位0.1º/s，输入范围[-1800, 1800]</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Roll轴角速度<br>单位0.1º/s，输入范围[-1800, 1800]</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Pitch轴角速度<br>单位0.1º/s，输入范围[-1800, 1800]</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td><ul>
    <li>7 bit</li>
    <li>1 bit</li>
    </ul></td>
  <td><ul>
    <li>保留字节</li>
    <li>是否启用</li>
    </ul></td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>

</table>

>备注：搭载A3的M600只支持对Ronin-MX/禅思X5系列/禅思X3/禅思XT进行云台角速度控制

#### 命令码 0x1B 云台角度控制
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>
<tr>
  <td rowspan="5">命令值</td>
  <td>0</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Yaw轴角度*<br>单位0.1º，输入范围 [-3200, 3200]</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Roll轴角度<br>单位0.1º，输入范围 [-350, 350]</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Pitch轴角度<br>单位0.1º，输入范围 [-900, 300]</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>---</td>
  <td>属性控制字节<ul>
    <li>bit 0:控制模式选择位</li>
        <ul>0 : 增量控制，角度基准为当前云台所处位置</ul>
        <ul>1 : 绝对控制，角度基准与手机DJI Go App设置有关*</ul>
    <li>bit 1:Yaw轴命令控制失效位 
        <ul>0 : 云台Yaw角度运动到命令位置 </ul>
        <ul>1 : 云台Yaw将维持上一时刻状态 </ul>
    <li>bit 2:Roll轴命令控制失效位，同bit1描述</li>
    <li>bit 3:Pitch轴命令控制失效位，同bit1描述</li>
    <li>bit 4:7:保留，设置为0</li>
    </ul></td>
<tr>
  <td>7</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>命令完成时间<br>单位0.1s，例如20代表云台在2s内匀速转动至命令位置<br>建议开发者控制速度不超过400º/秒</td>
</tr>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
  
**绝对控制模式下角度基准与手机DJI Go App中云台工作模式关系**  
<table>
  <tr>
    <th>云台工作模式</th>
    <th>Roll</th>
    <th>Pitch</th>
    <th>Yaw</th>
    <th>云台方向是否跟随机头</th>
  </tr>
  <tr>
    <td>跟随模式</td>
    <td>Ground</td>
    <td>Ground</td>
    <td>Body</td>
    <td align="center">是</td>
  </tr>
  <tr>
    <td>FPV模式</td>
    <td>不可控</td>
    <td>Ground</td>
    <td>不可控</td>
    <td align="center">是</td>
  </tr>
  <tr>
    <td>自由模式</td>
    <td>Ground</td>
    <td>Ground</td>
    <td>Ground</td>
    <td align="center">否</td>
  </tr>
</table>

**注意:将pitch旋转90度后会触发万向锁问题，此时roll与yaw的读数会突变。**

>备注：搭载A3的M600只支持对Ronin-MX/禅思X5系列/禅思X3/禅思XT进行云台角度控制

#### 命令码 0x20 相机拍照

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr> 
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
>备注：搭载A3的M600只支持对禅思系列相机进行拍照控制

#### 命令码 0x21 相机开始录像

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr> 
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>

</table>
>备注：搭载A3的M600只支持对禅思系列相机进行录像控制

#### 命令码 0x22 相机停止录像

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr> 
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
>备注：搭载A3的M600只支持对禅思系列相机进行录像控制

### 命令集 0x02 推送数据类

#### 命令码 0x00 飞行数据

飞控外发的状态数据包可以通过 DJI PC 调参软件配置。  
更多详情请参阅飞行数据详细说明请访问[附录](Appendix_cn.md#飞行数据说明)飞行数据说明部分。
<table>
<tr>
  <th>数据类型</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="13">命令值</td>
  <td>2</td>
  <td>状态包存在标志位，标志位为 1 表示推送数据中存在该状态包<br>M100:<ul>
    <li>bit 0:时间戳存在标志</li>
    <li>bit 1:姿态四元数存在标志</li>
    <li>bit 2:加速度存在标志</li>
    <li>bit 3:速度存在标志</li>
    <li>bit 4:角速度存在标志</li>
    <li>bit 5:GPS 位置、海拔（气压计数值）、相对地面高度、健康度存在标志</li>
    <li>bit 6:磁感计数值存在标志</li>
    <li>bit 7:遥控器通道值存在标志</li>
    <li>bit 8:云台 roll、pitch、yaw 数据存在标志</li>
    <li>bit 9:飞行状态存在标志</li>
    <li>bit 10:剩余电池百分比存在标志</li>
    <li>bit 11:控制设备存在标志</li>
    <li>bit 12:15:保留</li>
    </ul>
    A3:<ul>
    <li>bit 0:时间戳存在标志</li>
    <li>bit 1:姿态四元数存在标志</li>
    <li>bit 2:加速度存在标志</li>
    <li>bit 3:速度存在标志</li>
    <li>bit 4:角速度存在标志</li>
    <li>bit 5:GPS 位置、海拔（气压计数值）、相对地面高度、健康度存在标志</li>
    <li>bit 6:GPS 详细信息标志位</li>
    <li>bit 7:RTK 详细信息标志位</li>
    <li>bit 8:磁感计数值存在标志</li>
    <li>bit 9:遥控器通道值存在标志</li>
    <li>bit 10:云台 roll、pitch、yaw 数据存在标志</li>
    <li>bit 11:飞行状态存在标志</li>
    <li>bit 12:剩余电池百分比存在标志</li>
    <li>bit 13:控制设备存在标志</li>
    <li>bit 14:15:保留</li>
    </ul>
    </td>
</tr>

<tr>
  <td>9</td>
  <td>时间戳</td>
</tr>

<tr>
  <td>16</td>
  <td>姿态四元数</td>
</tr>

<tr>
  <td>12</td>
  <td>加速度</td>
</tr>

<tr>
  <td>13</td>
  <td>速度</td>
</tr>

<tr>
  <td>12</td>
  <td>角速度</td>
</tr>

<tr>
  <td>25</td>
  <td>GPS 位置, 海拔高度, 相对地面高度</td>
</tr>

<tr>
  <td>68</td>
  <td>GPS 详细信息（只适用于A3）</td>
</tr>

<tr>
  <td>74</td>
  <td>RTK 详细信息（只适用于A3）</td>
</tr>

<tr>
  <td>6</td>
  <td>磁感计数值</td>
</tr>

<tr>
  <td>12</td>
  <td>遥控器通道值</td>
</tr>

<tr>
  <td>13</td>
  <td>云台姿态</td>
</tr>

<tr>
  <td>1</td>
  <td>飞行状态</td>
</tr>

<tr>
  <td>1</td>
  <td>剩余电池百分比</td>
</tr>

<tr>
  <td>2</td>
  <td>控制设备</td>
</tr>
</table>
>备注：数据在推送数据中的偏移需要根据标志位确定存在的状态包，然后根据各状态包大小计算出实际偏移大小。

#### 命令码 0x01 失去控制权
机载设备的控制权优先级最低，其控制权可能在任何时候被夺去。  
当机载设备失去控制权时该数据包会由飞控发送。  


<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>固定值，0x04</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>

</table>
#### 命令码 0x03 地面站状态
更多信息请参考[地面站功能](GroundStationProtocol_cn.md)  
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>6</td>
  <td>地面站状态包</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
#### 命令码 0x04 航点事件
更多信息请参考[地面站功能](GroundStationProtocol_cn.md)  
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>6</td>
  <td>航点事件包</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
### 命令集 0x03 地面站功能类

更多信息请参考[地面站功能](GroundStationProtocol_cn.md)  

#### 命令码 0x10 上传航点任务信息

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>*</td>
  <td>waypoint_mission_info_comm_t</td>
  <td>任务信息</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xD1: 需要获取控制权</li>
    <li>0xD4: 飞机正在航点飞行，无法上传航线任务信息</li>
    <li>0xC8: 产品不支持航点功能</li>
    <li>0xE0: 航点信息参数不合理，超出范围</li>
  </ul></td>
</tr>
</table>

#### 命令码 0x11 上传航点信息
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="2">命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>航点索引</td>
</tr>
<tr>
  <td>1</td>
  <td>*</td>
  <td>waypoint_comm_t</td>
  <td>航点信息</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码: <ul>
    <li>0x00: 无错误 </li>
    <li>0xD4: 飞机正在航点飞行，无法上传航线任务信息</li>
    <li>0xDD: 有航线经过限飞区域</li>
    <li>0xE1: 存在航点任务不合法</li>
    <li>0xE2: 航点距离过长(15km)</li>
    <li>0xE4: 航点号超过最大任务航点数</li>
    <li>0xE5: 存在相邻航点太近</li>
    <li>0xE6: 存在相邻航点太远</li>
    <li>0xE7: 转弯处长度(damping_dis)距离检查失败</li>
    <li>0xE8: 航点动作参数不合法</li>
    <li>0xE9: 航点没有上传完成(仍有未上传的航点)</li>
    <li>0xEA: 航点任务信息没有上传</li>
  </ul></td>
</tr>
</table>
#### 命令码 0x12 启动/停止航点功能

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 : 停止航点功能<br>
    0x00 : 启动航点功能
  </td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xC0: 有航点高度超过限高高度</li>
    <li>0xC7: 有航点在限远半径外</li>
    <li>0xD7: 跟随功能正在运行</li>
    <li>0xD8: GPS信号弱</li>
    <li>0xD9: 电池电量低</li>
    <li>0xDA: 飞机不在空中</li>
    <li>0xDE: GPS信号弱，Home点没有成功记录</li>
    <li>0xE3: 航线总长度过长(15km)</li>
    <li>0xEA: 航点任务信息没有上传</li>
    <li>0xEB: 航点信息没有上传</li>
    <li>0xEC: 航点当前状态与请求相同，命令忽略</li>
    <li>0xF4: 命令码错误</li>
  </ul></td>
</tr>

</table>
#### 命令码 0x13 暂停/恢复航点功能

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 : 恢复航点功能<br>
    0x00 : 暂停航点功能
  </td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xED: 航点功能未执行</li>
    <li>0xF4: 命令码错误</li>
  </ul></td>
</tr>
</table>

#### 命令码 0x14 下载航点任务信息
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>任意值</td>
</tr>
<tr>
  <td rowspan="2">应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xEA: 航点任务信息没有上传</li>
  </ul></td>
</tr>
<tr>
  <td>1</td>
  <td>*</td>
  <td>waypoint_mission_info_comm_t</td>
  <td>任务信息</td>
</tr>
</table>
#### 命令码 0x15 下载航点信息
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>航点索引号</td>
</tr>
<tr>
  <td rowspan="3">应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xEB: 航点信息没有上传</li>
  </ul></td>
</tr>
<tr>
  <td>2</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>航点索引</td>
</tr>
<tr>
  <td>3</td>
  <td>*</td>
  <td>waypoint_commt_t</td>
  <td>航点信息</td>
</tr>
</table>
#### 命令码 0x16 设置巡航速度

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>*</td>
  <td>float32</td>
  <td>巡航速度</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xD1: 需要获取控制权</li>
    <li>0xEA: 航点任务信息没有上传</li>
    <li>0xEE: 巡航速度不合法</li>
  </ul></td>
</tr>
</table>
#### 命令码 0x17 读取巡航速度

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>任意值</td>
</tr>
<tr>
  <td rowspan="2">应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xD1: 需要获取控制权</li>
    <li>0xEA: 航点任务信息没有上传</li>
    </ul></td>
</tr>
<tr>
  <td>1</td>
  <td>2</td>
  <td>float32</td>
  <td>巡航速度</td>
</tr>
</table>
#### 命令码 0x20 启动热点功能
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>*</td>
  <td>hotpoint_mission_setting_t</td>
  <td>任务信息</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xA2: 输入参数不是有效的浮点数</li>
    <li>0xA3: 输入经纬度错误</li>
    <li>0xA6: 输入方向错误</li>
    <li>0xC0: 兴趣点高度太高</li>
    <li>0xC1: 兴趣点高度太低</li>
    <li>0xC2: 半径超出了允许范围(5m~500m)</li>
    <li>0xC3: 设置的速度太大</li>
    <li>0xC4: 切入点非法</li>
    <li>0xC5: 航向模式非法</li>
    <li>0xC7: 半径到达了限远距离</li>
    <li>0xC8: 不支持热点环绕</li>
    <li>0xC9: 飞机当前位置距兴趣点太远</li>
    <li>0xD3: 任务没有初始化</li>
    <li>0xD5: 正在进行热点环绕</li>
    <li>0xD6: 飞机从当前位置飞到切入点预计时间太长</li>
    <li>0xD7: 正在执行跟随或航点功能</li>
    <li>0xD8: GPS信号弱</li>
    <li>0xD9: 电池电量低</li>
    <li>0xDA: 飞机不在空中</li>
    <li>0xDC: 飞机正在起飞、降落或返航</li>
    <li>0xDD: 有航线经过限飞域</li>
    <li>0xDE: GPS信号弱，Home点没有成功记录</li>
    </ul></td>
</tr>
</table>

#### 命令码 0x21 停止热点功能

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>任意值</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误</li>
    <li>0xD1: 需要获取控制权</li>
    <li>0xD5: 热点功能未运行</li>
  </ul></td>
</tr>
</table>
#### 命令码 0x22 暂停/恢复热点功能

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 : 恢复热点功能<br>
    0x00 : 暂停热点功能
  </td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>返回码:<ul>
    <li>0x00: 无错误</li>
    <li>0xD5: 热点功能未运行</li>
    <li>0xA9: 热点功能已暂停</li>
    <li>0xAA: 热点功能未暂停 </li>
  </ul></td>
</tr>
</table>

#### 命令码 0x23 设置巡航速度

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="2 ">命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>顺逆时针</td>
</tr>
<tr>
  <td>1</td>
  <td>4</td>
  <td>float32</td>
  <td>巡航速度</td>
</tr>
<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
#### 命令码 0x24 设置半径
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>4</td>
  <td>float32</td>
  <td>半径</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>

#### 命令码 0x25 重置方向
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
#### 命令码 0x26 获取热点功能配置信息

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>任意值</td>
</tr>

<tr>
  <td rowspan="2">应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误</li>
    <li>0xD5: 热点功能未运行</li>
  </ul></td>
</tr>
<tr>
  <td>1</td>
  <td>*</td>
  <td>hotpoint_mission_setting_t</td>
  <td>任务信息</td>
</tr>
</table>

#### 命令码 0x27 启动自动半径模式

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="2">命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>enable(1为启动0为停止)</td>
</tr>

<tr>
  <td>1</td>
  <td>1</td>
  <td>int8_t</td>
  <td>rate(半径变化速度百分比)</td>
</tr>
<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误</li>
    <li>其他错误码</li>
  </ul></td>
</tr>
</table>

#### 命令码 0x30 启动跟随功能
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>*</td>
  <td>follow_me_mission_setting_t</td>
  <td>任务信息</td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xB0: 初始化时目标与飞机距离太大超过限制范围（20000m）</li>
    <li>0xC1: 高度太低</li>
    <li>0xC7: 初始化时目标与飞机距离太大超过最大飞行半径限制</li>
    <li>0xC8: 不支持跟随</li>
    <li>0xD1: 需要获取控制权</li>
    <li>0xD3: 任务没有初始化</li>
    <li>0xD5: 正在进行跟随</li>
    <li>0xD8: GPS信号弱</li>
    <li>0xD9: 电池电量低</li>
    <li>0xDA: 飞机不在空中</li>
    <li>0xDE: GPS信号弱，Home点没有成功记录</li>
  </ul></td>
</tr>
</table>
#### 命令码 0x31 停止跟随功能
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xD1: 需要获取控制权</li>
    <li>0xD4: 跟随功能未运行</li>
  </ul></td>
</tr>
</table>
#### 命令码 0x32 暂停/恢复跟随功能
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 : 恢复跟随功能<br>
    0x00 : 暂停跟随功能
  </td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>1</td>
  <td>返回码:<ul>
    <li>0x00: 无错误 </li>
    <li>0xD4: 跟随功能未运行</li>
  </ul></td>
</tr>
</table>

#### 命令码 0x33 设置跟踪目标点位置 
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答值</td>
  <td>0</td>
  <td>*</td>
  <td>cmd_mission_follow_target_info</td>
  <td>目标点位置</td>
</tr>
</table>
### 命令集 0x04 同步信号类
#### 命令码 0x00 设置同步信号输出频率
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>4</td>
  <td>uint32_t</td>
  <td>0表示单次同步，其他数值表示同步频率单位hz</td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
### 命令集 0x05 虚拟遥控类

更多信息请参考[虚拟遥控](VirtualRCProtocol_cn.md)

#### 命令码 0x00 虚拟遥控控制请求
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>描述</th>
</tr>

<tr>
  <td>命令值</td>
  <td>0</td>
  <td>1</td>
  <td>
    bit 0: <ul>1 请求打开<br>0 请求关闭</ul>
    bit 1: <ul>1 当断线时自动切换为真实遥控<br> 0 不切换，执行断线逻辑</ul>
    bit 2:7 <ul> 保留</ul></td>
</tr>

<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
#### 命令码 0x01 虚拟遥控数据
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>数据类型</th>
  <th>描述</th>
</tr>

<tr>
  <td rowspan="1">命令值</td>
  <td>0</td>
  <td>4 * 16</td>
  <td>uint32_t[16]</td>
  <td>十六通道数值，范围[1024-660， 1024+660]</td>
</tr>
<tr>
  <td>应答值</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>
</table>
[0]: Appendix_cn.md
