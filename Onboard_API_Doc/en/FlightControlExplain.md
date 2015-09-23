# Additional Explanation for Flight Control 

## Coordinate Frames

1. Body Frame

  ![bFrame](Images/axis.png)

2. Ground Frame
  + North - x axis
  + East - y axis
  + Down - z axis*

In general, in the ground frame, a general definition for the UAV orientation is North = 0 degree, East = 90 degree, West = -90 degree and South can be either 180 degree or -180 degree.

**The direction of ground frame is NOT natural for height control. So we adjust the direction of vertical control with the hope to make height and vertical velocity to be positive upwards. In other words, positive velocity makes the UAV ascend. This adjustment does not effect the directions and the orders of the other two axis.*

## Control mode byte

### Control mode byte

To control the movement of the UAV, control inputs can be divided into 3 parts including horizontal control, vertical control and yaw control. Each part has several sub modules.

The flag can be divided into 8 bits:
<table>
<tr>
  <td rowspan="5">Control mode byte</td>
  <td>bit[7:6]</td>
  <td>0b00: HORI_ATTI_TILT_ANG<br>0b01: HORI_VEL<br>0b10: HORI_POS</td>
</tr>
<tr>
  <td>bit[5:4]</td>
  <td>0b00: VERT_VEL<br>0b01: VERT_POS<br>0b10: VERT_THRUST</td>
</tr>
<tr>
  <td>bit[3]</td>
  <td>0b0: YAW_ANG<br>0b1: YAW_RATE</td>
</tr>
<tr>
  <td>bit[2:1]</td>
  <td>0b00: horizontal frame is ground frame<br>0b01: horizontal frame is body frame</td>
</tr>
<tr>
  <td>bit[0]</td>
  <td>0b0: yaw frame is ground frame<br>0b1: yaw frame is body frame</td>
</tr>
</table>

###Control mode  

We suggest developers do not use VERT_POS control mode indoor when your UAV does not have Guidance installed or the flight height is larget than 3 meters. Since in indoor environment, barometer can be inaccurate, the vertical controller may fail to keep the height of the UAV. 

> Please note that if the following conditions are met that the control mode is functional:
> 
* Only when the GPS signal is good (health\_flag >=3)，horizontal position control (HORI_POS) related control modes can be used.
* Only when GPS signal is good (health\_flag >=3)，or when Gudiance system is working properly (right connection and power provided)，horizontal velocity control（HORI_VEL）related control modes can be used.


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
  <td>HORI_ATTI_TILT_ANG*</td>
  <td>Pitch & roll angle, need to be referenced to either the ground or body frame</td>
</tr>
<tr>
  <td>HORI_POS**</td>
  <td>Position offsets of pitch & roll directions, need to be referenced to either the ground or body frame</td>
</tr>
<tr>
  <td>HORI_VEL</td>
  <td>Velocities on pitches & roll directions, need to be referenced to either the ground or body frame</td>
</tr>

<tr>
  <td rowspan="2">Yaw</td>
  <td>YAW_ANG</td>
  <td>Yaw angle is referenced to the ground frame</td>
</tr>
<tr>
  <td>YAW_RATE</td>
  <td>Yaw angular rate. It can either be referenced to either the ground frame or the body frame</td>
</tr>
</table>

<!-- **HORI_ATTI_TILT_ANG模式控制量如下图，DJI飞控采用水平面直接进行整个平面旋转。其中平面旋转角度为Θ,旋转方向与x轴或roll轴方向角度为γ。输入参量Θx=Θ*cos(γ),Θy=Θ*sin(γ)。(当采用Ground坐标系时γ为飞行方向与正北方向夹角，此时飞行器飞行状态与IOC模式相似；当采用Body坐标系时γ为飞行方向与飞行器机头方向夹角，此时飞行器飞行状态与遥控器下的姿态模式相似)* 

<div align="center">
<img src="Images/HORI_ATTI_TILT_ANG.jpg" alt="HORI_ATTI_TILT_ANG" width="540">
</div> -->


***The input of HORI_POS is a position offset instead of an actual position. This design aims to take both GPS flight and vision-based flight into consideration. If the developer wants to use GPS navigation, the GPS information sent by the UAV can be used to calculate position offset. While in vision-based flight application, developers should have their own positioning device (along with Gudiance or GPS to provide velocity measurement) to do position control. For example, [xuhao1 SDK package](https://github.com/xuhao1/dji_sdk/blob/master/src/modules/dji_services.cpp) realizes a GPS-based position control where target position can be passed as GPS coordinate.*



### Combinations  

By specifying the `control_mode_byte`, 14 control modes can be constructed :

|Index|Combinations|Input Data Range<br>(throttle/pitch&roll/yaw)|control_mode_byte*|
|---|------------|---------------------------------------------|--------------|
|1|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-18**0 degree ~ 180 degree|0b00000xx0|
|2**|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b00001xxy|
|3|VERT_VEL<br>HORI_VEL<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b01000xx0|
|4|VERT_VEL<br>HORI_VEL<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b01001xxy|
|5|VERT_VEL<br>HORI_POS<br>YAW_ANG|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b10000xx0|
|6|VERT_VEL<br>HORI_POS<br>YAW_RATE|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b10001xxy|
|7|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|0m to height limit<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b00010xx0|
|8|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|0m to height limit<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b00011xxy|
|9|VERT_POS<br>HORI_VEL<br>YAW_ANG|0m to height limit<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b01010xx0|
|10|VERT_POS<br>HORI_VEL<br>YAW_RATE|0m to height limit<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b01011xxy|
|11|VERT_POS<br>HORI_POS<br>YAW_ANG|0m to height limit<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b10010xx0|
|12|VERT_POS<br>HORI_POS<br>YAW_RATE|0m to height limit<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b10011xxy|
|13|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b00100xx0|
|14|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b00101xxy|


**Low 3 bits in control_mode_byte decides the horizontal frame and yaw frame in some combinations.*  
*xx presents horizontal frame，00 means ground frame，01 means body frame.*  
*y presents yaw frame，0 means ground frame，1 means body frame*

***In this combination，if all input data is '0', the UAV will brake, which can also keep its balance in air.*

## Flight status
### Flight status
|Flight status|status name|description| 
|-------|-------|---|
|1|standby|UAV standby|
|2|take_off|UAV take off|
|3|in_air|UAV in air|
|4|landing|UAV landing|
|5|post_landing|UAV landing finish|

*Note: We recommend use sensors' data to control UAV, instead of just flight status.*
