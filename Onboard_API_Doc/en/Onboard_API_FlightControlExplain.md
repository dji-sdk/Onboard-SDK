# Additional Explanation for Flight Control 

*If you come across any mistakes or bugs in this tutorial, please let us know using a Github issue, a post on the DJI forum, or email. Please feel free to send us Github pull request and help us fix any issues.*

## Explanation of Coordinate Frames

1. Body Frame

  ![bFrame](Images/axis.png)

2. Ground Frame
  + North - x axis
  + East - y axis
  + Down - z axis

Therefore, in the ground frame, a general definition for craft orientation is North = 0 degree, East = 90 degree, West = -90 degree and South can be either 180 degree or -180 degree.

**Note: The direction of ground frame is not natural for height control. So we adjust the direction of vertical control in order to make height and vertical velocity to be positive upwards, in other words, positive velocity makes MATRICE 100 ascend. This adjustment does not change the direction and the order of the other two axis.**

## Explanation of ctrl mode flag

To control the spatial movement of MATRICE 100, we split control inputs into three parts: horizontal control, vertical control and yaw control. Each part has several sub modules.

<table>
<tr>
  <th>Category</th>
  <th>Mode</th>
  <th>Explanation</th>
</tr>
<tr>
  <td rowspan="3">Vertical</td>
  <td>VERT_POS</td>
  <td>Control the height of MATRICE 100</td>
</tr>
<tr>
  <td>VERT_VEL</td>
  <td>Control the vertical speed of MATRICE 100, upward is positive</td>
</tr>
<tr>
  <td>VERT_THRUST</td>
  <td>Directly control the thrust (lifting force expressed as 0%-100% percentage) of MATRICE 100</td>
</tr>

<tr>
  <td rowspan="3">Horizontal</td>
  <td>HORI_ATTI_TILT_ANG</td>
  <td>Pitch & roll angle, <b>referenced to either the ground or body frame</b></td>
</tr>
<tr>
  <td>HORI_POS</td>
  <td>Position offsets of pitch & roll directions, <b>referenced to either the ground or body frame</b></td>
</tr>
<tr>
  <td>HORI_VEL</td>
  <td>Velocities on pitches & roll directions, <b>referenced to either the ground or body frame</b></td>
</tr>

<tr>
  <td rowspan="2">Yaw</td>
  <td>YAW_ANG</td>
  <td>Yaw angle referenced to the ground frame</td>
</tr>
<tr>
  <td>YAW_RATE</td>
  <td>Yaw angular rate. It can either be <b>referenced to either the ground frame or the body frame</b></td>
</tr>
</table>

The ctrl mode flag is divided into 8 bits:
<table>
<tr>
  <td rowspan="5">ctrl_mode_flag<br>1byte</td>
  <td>bit[7:6]</td>
  <td>0b00: horizontal angle<br>0b01: horizontal velocity<br>0b10:horizontal position</td>
</tr>
<tr>
  <td>bit[5:4]</td>
  <td>0b00: vertical velocity<br>0b01: vertical position<br>0b10: vertical thrust</td>
</tr>
<tr>
  <td>bit[3]</td>
  <td>0b0: yaw angle<br>0b1: yaw angular rate</td>
</tr>
<tr>
  <td>bit[2:1]</td>
  <td>0b00: horizontal frame is ground frame<br>0b01: horizontal frame is body frame</td>
</tr>
<tr>
  <td>bit[0]</td>
  <td>0b0: yaw frame is ground frame<br>0b1: yaw frame is body frame</td>
</tr>
<table>

`HORI_FRAME` and `YAW_FRAME` can be an arbitrary if the corresponding mode does not need a specify frame. 

By specifying `ctrl_mode_flag`, 14 control modes can be constructed (`ctrl_mode_flag` is represented as an 8-bit binary number here. The bit position with X means that this certain mode doesn't depend on the bit of this position, it can be either 0 or 1. Here "0b" means we represent the flag with binary number, the last 8 bits constructs a 0-255 integer):

|No.|Combinations|Input Data Range<br>(throttle/pitch&roll/yaw)|ctrl_mode_flag|
|---|------------|---------------------------------------------|--------------|
|1|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b000000XX|
|2|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b000010XX|
|3|VERT_VEL<br>HORI_VEL<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b010000XX|
|4|VERT_VEL<br>HORI_VEL<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b010010XX|
|5|VERT_VEL<br>HORI_POS<br>YAW_ANG|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b100000XX|
|6|VERT_VEL<br>HORI_POS<br>YAW_RATE|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b100010XX|
|7|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|0m to height limit<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b000100XX|
|8|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|0m to height limit<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b000110XX|
|9|VERT_POS<br>HORI_VEL<br>YAW_ANG|0m to height limit<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b010100XX|
|10|VERT_POS<br>HORI_VEL<br>YAW_RATE|0m to height limit<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b010110XX|
|11|VERT_POS<br>HORI_POS<br>YAW_ANG|0m to height limit<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b100100XX|
|12|VERT_POS<br>HORI_POS<br>YAW_RATE|0m to height limit<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b100110XX|
|13|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b001000XX|
|14|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b001010XX|

The input of HORI_POS is a position offset instead a actual position. This design aims to take both GPS flight and vision-based flight into account. If the developer wants to use GPS navigation, the GPS information sent by Matrice 100 can be used to calculate position offset. While in vision-based flight application, developers should have their own positioning device (along with Gudiance or GPS to provide velocity measurement) to do position control. For example, [xuhao1 SDK package](https://github.com/xuhao1/dji_sdk/blob/master/src/modules/dji_services.cpp) realizes a GPS-based position control where target position can be passed as GPS coordinate.

We suggest developers do not user VERT_POS control mode indoor when Matrice 100 does not have Guidance installed or the flight height is larget than 3 meters. Because in indoor environment, barometer can be very inaccurate, the vertical controller may fail to keep the height of Matrice 100. 

**CAUTION！VERY IMPORTANT：control mode has entering conditions：**

- Only when GPS signal is good (health\_flag >=3)，horizontal **position** control (HORI_POS) related control modes can be used.
- Only when GPS signal is good (health\_flag >=3)，or when Gudiance system is working properly (right connection and power provided)，horizontal **velocity** control（HORI_VEL）related control modes can be used.
