#### This documentation is now deprecated, please refer to <https://developer.dji.com/onboard-sdk/documentation/appendix/index.html> in DJI Developer Website.

# Appendix

## Coordinate Frames

1. Body Frame

  ![bFrame](Images/axis.png)

2. Ground Frame
  
  + North - x axis
  + East - y axis
  + Down - z axis*

In general, in the ground frame, a general definition for the UAV orientation is North = 0 degree, East = 90 degree, West = -90 degree and South can be either 180 degree or -180 degree.

**The direction of ground frame is NOT natural for the presentation of height. For this, we adjust the direction of vertical control in order to make the height or vertical velocity to be positive upwards. In other words, given a positive velocity will make the UAV ascend. This adjustment does not effect the directions and the orders of the other two axis.*

## Control mode byte

### Control mode byte

3 parts of control inputs can be used to control the movement of the UAV including horizontal control, vertical control and yaw control. Each part has several sub modules.

<table>
<tr>
  <td rowspan="5">Control mode byte</td>
  <td>bit 7:6</td>
  <td>0b00: HORI_ATTI_TILT_ANG<br>0b01: HORI_VEL<br>0b10: HORI_POS</td>
</tr>
<tr>
  <td>bit 5:4</td>
  <td>0b00: VERT_VEL<br>0b01: VERT_POS<br>0b10: VERT_THRUST</td>
</tr>
<tr>
  <td>bit 3</td>
  <td>0b0: YAW_ANG<br>0b1: YAW_RATE</td>
</tr>
<tr>
  <td>bit 2:1 </td>
  <td>0b00: horizontal frame is ground frame<br>0b01: horizontal frame is body frame</td>
</tr>
<tr>
  <td>bit 0 </td>
  <td>0b0: non-stable mode<br>0b1: stable mode</td>
</tr>
</table>

###Control mode  

We suggest developers do not use VERT_POS control mode indoor when your UAV does not have Guidance installed or the flight height is larger than 3 meters. Since in indoor environment, barometer can be inaccurate, the vertical controller may fail to keep the height of the UAV. 

> Please note that if the following conditions are met that the control mode is functional:
> 
* Only when the GPS signal is good (health\_flag >=3)，horizontal position control (HORI_POS) related control modes can be used.
* Only when GPS signal is good (health\_flag >=3)，or when Gudiance system is working properly with Autopilot，horizontal velocity control（HORI_VEL）related control modes can be used.


<table>
<tr>
  <th>Category</th>
  <th>Mode</th>
  <th>Explanation</th>
</tr>
<tr>
  <td rowspan="3">Vertical</td>
  <td>VERT_POS</td>
  <td>Control the height of UAV</td>
</tr>
<tr>
  <td>VERT_VEL</td>
  <td>Control the vertical speed of UAV, upward is positive</td>
</tr>
<tr>
  <td>VERT_THRUST</td>
  <td>Directly control the thrust, the range is from 0% to 100%</td>
</tr>

<tr>
  <td rowspan="3">Horizontal</td>
  <td>HORI_ATTI_TILT_ANG</td>
  <td>Pitch & roll angle, need to be referenced to either the ground or body frame</td>
</tr>
<tr>
  <td>HORI_POS*</td>
  <td>Position offsets of pitch & roll directions, need to be referenced to either the ground or body frame</td>
</tr>
<tr>
  <td>HORI_VEL</td>
  <td>Velocities on pitch & roll directions, need to be referenced to either the ground or body frame</td>
</tr>

<tr>
  <td rowspan="2">Yaw</td>
  <td>YAW_ANG</td>
  <td>Yaw angle is referenced to the ground frame. In this control mode, Ground frame is enforeced in Autopilot</td>
</tr>
<tr>
  <td>YAW_RATE</td>
  <td>Yaw angular velocity</td>
</tr>
</table>

<!-- **HORI_ATTI_TILT_ANG模式控制量如Down图，DJI飞控采用水平面直接进行整个平面旋转。其中平面旋转角度为Θ,旋转方向与x轴或roll轴方向角度为γ。输入参量Θx=Θ*cos(γ),Θy=Θ*sin(γ)。(当采用Ground坐标系时γ为飞行方向与正北方向夹角，此时飞行器飞行状态与IOC模式相似；当采用Body坐标系时γ为飞行方向与飞行器机头方向夹角，此时飞行器飞行状态与遥控器Down的姿态模式相似)* 

<div align="center">
<img src="Images/HORI_ATTI_TILT_ANG.jpg" alt="HORI_ATTI_TILT_ANG" width="540">
</div> -->


**The input of HORI_POS is a position offset instead of an actual position. This design aims to take both GPS flight and vision-based flight into consideration. If the developer wants to use GPS navigation, the GPS information sent by the UAV can be used to calculate position offset. While in vision-based flight application, developers should have their own positioning device (along with Gudiance or GPS to provide velocity measurement) to do position control. **



### Combinations  

Attitude control accuracy is about 0.5 degrees, speed control accuracy of about 0.2 m/s.

|Index|Combinations|Input Data Range<br>(throttle/pitch&roll/yaw)|control_mode_byte|
|---|------------|---------------------------------------------|--------------|
|1|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b00000xxy|
|2|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b00001xxy|
|3|VERT_VEL<br>HORI_VEL<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b01000xxy|
|4|VERT_VEL<br>HORI_VEL<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b01001xxy|
|5|VERT_VEL<br>HORI_POS<br>YAW_ANG|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b10000xxy|
|6|VERT_VEL<br>HORI_POS<br>YAW_RATE|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b10001xxy|
|7|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|0m to height limit<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b00010xxy|
|8|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|0m to height limit<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b00011xxy|
|9|VERT_POS<br>HORI_VEL<br>YAW_ANG|0m to height limit<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b01010xxy|
|10|VERT_POS<br>HORI_VEL<br>YAW_RATE|0m to height limit<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b01011xxy|
|11|VERT_POS<br>HORI_POS<br>YAW_ANG|0m to height limit<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b10010xxy|
|12|VERT_POS<br>HORI_POS<br>YAW_RATE|0m to height limit<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b10011xxy|
|13|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b00100xxy|
|14|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b00101xxy|



>xx presents horizontal frame，00 means ground frame，01 means body frame.  
>y presents stable flag，0 means non-stable mode，1 means stable mode. Stable mode only works in horizontal control.  



## Flight Data

### Flight Data

<table>
<tr>
  <th>Item Name</th>
  <th>Variables</th>
  <th>Data Type</th>
  <th>Description</th>
  <th>Unit</th>
  <th>Default Frequency</th>
</tr>

<tr>
  <td rowspan="3">Time Stamp</td>
  <td>time</td>
  <td>uint32_t</td>
  <td>400hz time stamp</td>
  <td>1/400s</td>
  <td rowspan="3 ">100Hz</td>
</tr>
<tr>
  <td>time</td>
  <td>uint32_t</td>
  <td>ns time stamp</td>
  <td>ns</td>
</tr>
<tr>
  <td>time</td>
  <td>uint8_t</td>
  <td>Sync signal flag</td>
  <td>---</td>
</tr>
<tr>
  <td rowspan="4">Quarternion</td>
  <td>q0</td>
  <td>float32</td>
  <td rowspan="4">Attitude quaternion<br>From ground frame to body frame</td>
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
  <td rowspan="3">Linear acceleration</td>
  <td>agx</td>
  <td>float32</td>
  <td rowspan="3">Linear acceleration (Raw/Fusion)</td>
  <td rowspan="3">Fusion: m/s<sup>2</sup><br>Raw: G</td>
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
  <td rowspan="4">Linear velocity</td>
  <td>vgx</td>
  <td>float32</td>
  <td rowspan="3">Linear velocity in Ground Frame</td>
  <td rowspan="3">m/s</td>
  <td rowspan="4">100Hz</td>
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
  <td>vgstatus</td>
  <td>uint8_t</td>
  <td>Status byte of linear velocity<ul>
  <li>bit 0：data valid flag</li>
    <ul>0：invalid</ul>
    <ul>1：valid</ul>
  <li>bit 1:7 ：reserved</li>
  </ul></td>
  <td>---</td>
</tr>

<tr>
  <td rowspan="3">Angular velocity</td>
  <td>wx</td>
  <td>float32</td>
  <td rowspan="3">Angular velocity (Raw/Fusion)</td>
  <td rowspan="3">rad/s</td>
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
  <td rowspan="5">GPS and altitude</td>
  <td>longti</td>
  <td>double</td>
  <td rowspan="2">GPS location</td>
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
  <td>Altitude (Raw/Fusion)</td>
  <td>m</td>
</tr>
<tr>
  <td>height</td>
  <td>float32</td>
  <td>Height relatively to ground (Raw/Fusion)</td>
  <td>m</td>
</tr>
<tr>
  <td>health_flag</td>
  <td>uint8_t</td>
  <td>GPS healthiness </td>
  <td>0-5, 5 is the best condition</td>
</tr>
<tr>
  <td rowspan="8"> GPS detailed info (A3 only)</td>
  <td>date</td>
  <td>uint32_t</td>
  <td>date</td>
  <td>yy-mm-dd</td>
  <td rowspan="8">50Hz</td>
</tr>
<tr>
  <td>time</td>
  <td>uint32_t</td>
  <td>time</td>
  <td>hh-mm-ss</td>
</tr>
<tr>
  <td>longitude</td>
  <td>int32_t</td>
  <td>longitude</td>
  <td>degree*10^7</td>
</tr>
<tr>
  <td>latitude</td>
  <td>int32_t</td>
  <td>latitude</td>
  <td>degree*10^7</td>
</tr>
<tr>
  <td>AMSL</td>
  <td>int32_t</td>
  <td>height above mean sea level</td>
  <td>mm</td>
</tr>
<tr>
  <td>vel_N</td>
  <td>float32</td>
  <td>velocity in North direction</td>
  <td>cm/s</td>
</tr>
<tr>
  <td>vel_E</td>
  <td>float32</td>
  <td>velocity in East direction</td>
  <td>cm/s</td>
</tr>
<tr>
  <td>vel_D</td>
  <td>float32</td>
  <td>velocity in Down direction</td>
  <td>cm/s</td>
</tr>
<tr>
  <td rowspan="11"> RTK detailed info (A3 only)</td>
  <td>date</td>
  <td>uint32_t</td>
  <td>date</td>
  <td>yy-mm-dd</td>
  <td rowspan="11">50Hz</td>
</tr>
<tr>
  <td>time</td>
  <td>uint32_t</td>
  <td>time</td>
  <td>hh-mm-ss</td>
</tr>
<tr>
  <td>longitude_RTK</td>
  <td>double</td>
  <td>longitude by RTK</td>
  <td>degree</td>
</tr>
<tr>
  <td>latitude_RTK</td>
  <td>double</td>
  <td>latitude by RTK</td>
  <td>degree</td>
</tr>
<tr>
  <td>AMSL_RTK</td>
  <td>float32</td>
  <td>AMSL by RTK</td>
  <td>m</td>
</tr>
<tr>
  <td>vel_N</td>
  <td>float32</td>
  <td>velocity in North direction</td>
  <td>cm/s</td>
</tr>
<tr>
  <td>vel_E</td>
  <td>float32</td>
  <td>velocity in East direction</td>
  <td>cm/s</td>
</tr>
<tr>
  <td>vel_D</td>
  <td>float32</td>
  <td>velocity in Down direction</td>
  <td>cm/s</td>
</tr>
<tr>
  <td>yaw</td>
  <td>int16_t</td>
  <td>angle between antenna baseline and South</td>
  <td>degree</td>
</tr>
  <td>position_flag</td>
  <td>uint8_t</td>
  <td>positioning flag</td>
  <td></td>
</tr>
<tr>
  <td>yaw_flag</td>
  <td>uint8_t</td>
  <td>yaw flag</td>
  <td>---</td>
</tr>
<tr>
  <td rowspan="3">Magnetometer</td>
  <td>mx</td>
  <td>int16_t</td>
  <td rowspan="3">Magnetometer data</td>
  <td rowspan="3">Magnetometer data</td>
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
  <td rowspan="6">Remote controller channel</td>
  <td>roll</td>
  <td>int16_t</td>
  <td>roll channel</td>
  <td rowspan="6">---</td>
  <td rowspan="6">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>int16_t</td>
  <td>pitch channel</td>
</tr>
<tr>
  <td>yaw</td>
  <td>int16_t</td>
  <td>yaw channel</td>
</tr>
<tr>
  <td>throttle</td>
  <td>int16_t</td>
  <td>throttle channel</td>
</tr>
<tr>
  <td>mode</td>
  <td>int16_t</td>
  <td>mode channel</td>
</tr>
<tr>
  <td>gear</td>
  <td>int16_t</td>
  <td>gear channel</td>
</tr>

<tr>
  <td rowspan="4">Gimbal</td>
  <td>roll</td>
  <td>float32</td>
  <td rowspan="3">roll, pitch and yaw of ground frame</td>
  <td rowspan="3">º</td>
  <td rowspan="4">50Hz</td>
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
  <td>limit_byte</td>
  <td>uint8_t</td>
  <td>limit flag<ul>
    <li>bit 0: Pitch limit flag</li>
    <li>bit 1: Roll limit flag</li>
    <li>bit 2: Yaw limit flag</li>
    <li>bit 3:7 reserved</li></ul>
  <td>---</td>
</tr>
<tr>
  <td>Flight status</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Flight status</td>
  <td>---</td>
  <td>10Hz</td>
</tr>

<tr>
  <td>Battery</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Battery percentage</td>
  <td>%</td>
  <td>1Hz</td>
</tr>

<tr>
  <td rowspan="2">Source of Control</td>
  <td>cur_mov_control_mode</td>
  <td>uint8_t</td>
  <td>current api control mode</td>
  <td>---</td>
  <td rowspan="2">0Hz</td>
</tr>
<tr>
  <td>status</td>
  <td>uint8_t</td>
  <td>Control device<ul>
     <li>bit 0:2 ：Control device</li>
     <ul>0b000 ：Remote Controller</ul>
     <ul>0b001 ：Mobile Device</ul>
     <ul>0b010 ：Onboard Device</ul>
     <li>bit 3 ：Flag of Onboard Device control authorization request signature </li>
     <ul>0：No request</ul>
     <ul>1：Been requested</ul>
     <li>bit 4 ：Flag of Virtual RC </li>
     <ul>0：Disabled</ul>
     <ul>1：Enabled</ul>
     <li>bit 5:7 ：reserved</li>
  </ul></td>
  <td>---</td>

</tr>

</table>

**current api control mode**
~~~c
typedef enum 
{
  ATTI_STOP                    = 0,
  HORIZ_ANG_VERT_VEL_YAW_ANG   = 1,
  HORIZ_ANG_VERT_VEL_YAW_RATE  = 2,
  HORIZ_VEL_VERT_VEL_YAW_ANG   = 3,
  HORIZ_VEL_VERT_VEL_YAW_RATE  = 4,
  HORIZ_POS_VERT_VEL_YAW_ANG   = 5,
  HORIZ_POS_VERT_VEL_YAW_RATE  = 6,
  HORIZ_ANG_VERT_POS_YAW_ANG   = 7,
  HORIZ_ANG_VERT_POS_YAW_RATE  = 8,
  HORIZ_VEL_VERT_POS_YAW_ANG   = 9,
  HORIZ_VEL_VERT_POS_YAW_RATE  = 10,
  HORIZ_POS_VERT_POS_YAW_ANG   = 11,
  HORIZ_POS_VERT_POS_YAW_RATE  = 12,
  HORIZ_ANG_VERT_THR_YAW_ANG   = 13,
  HORIZ_ANG_VERT_THR_YAW_RATE  = 14,
  HORIZ_VEL_VERT_THR_YAW_ANG   = 15,
  HORIZ_VEL_VERT_THR_YAW_RATE  = 16,
  HORIZ_POS_VERT_THR_YAW_ANG   = 17,
  HORIZ_POS_VERT_THR_YAW_RATE  = 18,
  GPS_ATII_CTRL_CL_YAW_RATE    = 97,
  GPS_ATTI_CTRL_YAW_RATE       = 98,
  ATTI_CTRL_YAW_RATE           = 99,
  ATTI_CTRL_STOP               = 100
}
~~~
### Raw/Fusion

Raw/Fusion can be chosen by DJI assistant software.  

Because raw data is generated from actual sensor on UAV, this kind of data will not be available in simulator. Please choose Fusion when you use DJI simulator.  
<table>
<tr>
  <th>Item Name</th>
  <th>Raw/Fusion</th>
  <th>Description</td>
  <th>Unit</td>
</tr>
<tr>
  <td rowspan="3">Linear acceleration</td>
  <td>Fusion(Ground)</td>
  <td>Fusion data</td>
  <td>m/s<sup>2</sup></td>
</tr>
<tr>
  <td>Fusion(Body)</td>
  <td>Fusion data</td>
  <td>m/s<sup>2</sup></td>
</tr>
<tr>
  <td>Raw(Body)</td>
  <td>Accelerometer data</td>
  <td>G</td>
</tr>
<tr>
  <td rowspan="2">Angular velocity</td>
  <td>Fusion(Body)</td>
  <td>Fusion data</td>
  <td rowspan="2">rad/s</td>
</tr>
<tr>
  <td>Raw(Body)</td>
  <td>Gyro data</td>

</tr>
<tr>
  <td rowspan="2">Altitude</td>
  <td>Fusion</td>
  <td>Barometer & IMU </td>
  <td rowspan="2">m</td>
</tr>
<tr>
  <td>Raw</td>
  <td>Barometer data</td>
</tr>
<tr>
  <td rowspan="2">Height*</td>
  <td>Fusion</td>
  <td>Barometer、IMU & Ultrasound</td>
  <td rowspan="2">m</td>
</tr>
<tr>
  <td>Raw</td>
  <td>Ultrasound data（within three meters vaild）</td>

</tr>
</table>

>If the flight plantform has no ultrasonic sensor, or its distance to the ground is higher than 3 meters, the height is supported by barometer and IMU only. Since the barometer is inaccurate being used indoor, height is unreliable in this case.

### Flight status
|Flight status val|status name| 
|-------|-------|
|1|standby|
|2|take_off|
|3|in_air|
|4|landing|
|5|finish_landing|

>flight status will enter the 'standby' state after 2s in 'finish_landing state'.   
>flight status will immediately enter 'in_air' state when UAV leave the ground.

### Remote controller channel
|Channel|Range|Description|  
|-------|-------|---|
|roll|[-10000,10000]|Left: -10000<br>Right: 10000|
|pitch|[-10000,10000]|Down: -10000<br>Up: 10000|
|yaw|[-10000,10000]|Left: -10000<br>Right: 10000|
|throttle|[-10000,10000]|Down: -10000<br>Up: 10000|
|mode|-8000, 0, 8000|P: -8000<br>A: 0<br>F: 8000|
|gear|-10000, -4545|Gear down: -4545<br>Gear up: -10000|
