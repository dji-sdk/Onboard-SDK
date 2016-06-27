#### This documentation is now deprecated, please refer to <https://developer.dji.com/onboard-sdk/documentation/introduction/ground-station-protocol.html> in DJI Developer Website.

# Ground Station Protocol

This part is about Goundstation related functions(Waypoint, Hotpoint and Follow Me), which has been introduced briefly in the [OPEN Protocol](OPENProtocol.md#cmd-set-0x03-ground-station-cmd-set).

For the detialed function logic, please refer to the [Ground Station Programming Guide](GroundStationProgrammingGuide.md).

## Command Set and Command Id

<table>
    <tr>
        <th>CMD Set</th>
        <th>CMD Group</th>
        <th>CMD ID</th>
        <th>Description</th>
    </tr>
    <tr>
        <th rowspan="19">0x03</th>
        <th rowspan="8">Waypoint</th>
        <th>0x10</th>
        <th>upload waypoint task data</th>
    </tr>
    <tr>
        <th>0x11</th>
        <th>upload the waypoint data with certain index</th>
    </tr>
    <tr>
        <th>0x12</th>
        <th>start/stop waypoint mission</th>
    </tr>
    <tr>
        <th>0x13</th>
        <th>pause/resume waypoint mission</th>
    </tr>
    <tr>
        <th>0x14</th>
        <th>download waypoint task data</th>
    </tr>
    <tr>
        <th>0x15</th>
        <th>download certain waypoint data with given index</th>
    </tr>
    <tr>
        <th>0x16</th>
        <th>set waypoint mission idle velocity</th>
    </tr>
    <tr>
        <th>0x17</th>
        <th>read waypoint mission idle velocity</th>
    </tr>
    <tr>
        <th rowspan="8">Hotpoint</th>
        <th>0x20 </th>
        <th>upload hotpoint data and start hotpoint mission</th>
    </tr>
    <tr>
        <th>0x21</th>
        <th>stop hotpoint mission</th>
    </tr>
    <tr>
        <th>0x22</th>
        <th>pause/resume hotpoint mission</th>
    </tr>
    <tr>
        <th>0x23</th>
        <th>set hotpoint mission idle velocity</th>
    </tr>
    <tr>
        <th>0x24</th>
        <th>set hotpoint mission radius</th>
    </tr>
    <tr>
        <th>0x25</th>
        <th>reset yaw of hotpoint mission</th>
    </tr>
    <tr>
        <th>0x26</th>
        <th>download hotpoint mission data</th>
    </tr>
    <tr>
        <th>0x27</th>
        <th>enable auto-radiu mode</th>
    </tr>
    <tr>
        <th rowspan="4">Follow me</th>
        <th>0x30</th>
        <th>upload follow me data and start</th>
    </tr>
    <tr>
        <th>0x31</th>
        <th>stop follow me mission</th>
    </tr>
    <tr>
        <th>0x32</th>
        <th>pause/resume follow me mission</th>
    </tr>
    <tr>
        <th>0x33</th>
        <th>upload the target data of follow me</th>
    </tr>
    <tr>
        <th>0x02</th>
        <th>Mission Status Push Info</th>
        <th>0x03</th>
        <th>current mission status</th>
    </tr>
    <tr>
        <th>0x02</th>
        <th>Waypoint Mission Event Push Info</th>
        <th>0x04</th>
        <th>mission event</th>
    </tr>
</table>


## Data Structure

**Note:** All reserved bytes in ground station structs should be set as 0, otherwise your commands may fail.

---

### 0x03, 0x10: upload waypoint task data

Request:
```c
typedef struct {

    uint8_t length;//count of waypoints
    float vel_cmd_range;//Maximum speed joystick input(2~15m)
    float idle_vel;//Cruising Speed (without joystick input, no more than vel_cmd_range)
    uint8_t action_on_finish;//Action on finish
                            //0: no action
                            //1: return to home
                            //2: auto landing
                            //3: return to point 0
                            //4: infinite mode， no exit
    uint8_t mission_exec_num;//Function execution times
                            //1: once
                            //2: twice
    uint8_t yaw_mode;//Yaw mode
                    //0: auto mode(point to next waypoint)
                    //1: Lock as an initial value
                    //2: controlled by RC
                    //3: use waypoint's yaw(tgt_yaw)
    uint8_t trace_mode;//Trace mode
                    //0: point to point, after reaching the target waypoint hover, complete waypoints action (if any), then fly to the next waypoint
                    //1: Coordinated turn mode, smooth transition between waypoints, no waypoints task
    uint8_t action_on_rc_lost;//Action on rc lost
                            //0: exit waypoint and failsafe
                            //1: continue the waypoint
    uint8_t gimbal_pitch_mode;//Gimbal pitch mode
                            //0: Free mode, no control on gimbal
                            //1: Auto mode, Smooth transition between waypoints
    double hp_lati;//Focus latitude (radian)
    double hp_longti;//Focus longitude (radian)
    float hp_alti;//Focus altitude (relative takeoff point height)
    uint8_t resv[16];//reserved, must be set as 0
    
}waypoint_mission_info_comm_t;
```

ACK: `uint8_t`

**Note:** The ACK of task upload is always 0. Developers should check the task parameter by themselves, otherwise error code 0xEA will appear when trying to upload waypoints' data.

### 0x03, 0x11: upload the waypoint data with certain index

Request:
```c
typedef struct {
    double latitude;//waypoint latitude (radian)
    double longitude;//waypoint longitude (radian)
    float altitude;//waypoint altitude (relative altitude from takeoff point)
    float damping_dis;//bend length (effective coordinated turn mode only)
    int16_t tgt_yaw;//waypoint yaw (degree)
    int16_t tgt_gimbal_pitch;//waypoint gimbal pitch
    uint8_t turn_mode;//turn mode
                    //0: clockwise
                    //1: counter-clockwise
    uint8_t resv[8];//reserved
    
    uint8_t has_action;//waypoint action flag
                    //0: no action
                    //1: has action
    uint16_t action_time_limit;//waypoint action time limit unit:s
    waypoint_action_comm_t action;//waypoint action

} waypoint_comm_t

```

ACK:
```c
struct waypoint_upload_ack{
    uint8_t ack;
    uint8_t index;
};
```
    

For the waypoint action:

```c
typedef struct {
    uint8_t action_num :4;//total number of actions
    uint8_t action_rpt :4;//total running times
    
    uint8_t command_list[15];//command list, 15 at most
    int16_t command_param[15];//command param, 15 at most

}waypoint_action_comm_t
```

There are totally six kinds of actions as follows, which should be set in `command_list`.


|Commands|Commands value|Command param|Description|
|----|------|--------|----|
|WP_ACTION_STAY|0|Hover time unit: **milli**second|Just hover|
|WP_ACTION_SIMPLE_SHOT|1|N/A|Take a photo|
|WP_ACTION_VIDEO_START|2|N/A|Start record|
|WP_ACTION_VIDEO_STOP|3|N/A|Stop record|
|WP_ACTION_CRAFT_YAW|4|YAW (-180~180)|Adjust the aircraft toward|
|WP_ACTION_GIMBAL_PITCH|5|PITCH|Adjust gimbal pitch 0: head -90: look down|

**Note: ** The controller will valid all waypoints' data together after the last one uploaded, which means if there exist at least one invalid waypoint information, the waypoint upload ACK of the last one will be with a error code. 

### 0x03, 0x12: start/stop waypoint mission

Request:

```c
uint8_t start;//0-> start, 1-> cancel
```

ACK: `uint8_t`

### 0x03, 0x13: pause/resume waypoint mission

Request:

```c
uint8_t pause;//0-> pause, 1-> resume
```

ACK: `uint8_t`


### 0x03, 0x14: download waypoint task

Request:

`uint8_t` with arbitrary value.

ACK:

```c
struct waypoint_task_download_ack{
    uint8_t ack;
    waypoint_mission_info_comm_t wp_task;//defined in previous part
};
```

### 0x03, 0x15: download a certain waypoint

Request:

```c
uint8_t index;
```

ACK:

```c
struct waypoint_download_ack{
    uint8_t ack;
    uint8_t index;
    waypoint_comm_t wp_data;//defined in previous
};
```

### 0x03, 0x16: set idle velocity

Request:

```c
float idle_veloctity;
```

ACK:

```c
struct waypoint_set_vel_ack{
    uint8_t ack;
    float idle_velcity;
};
```

### 0x03, 0x17: read idle velocity

Request:

`uint8_t` with arbitrary value

ACK:

```c
struct waypoint_read_vel_ack{
    uint8_t ack;
    float idle_velcity;
};
```


### 0x03, 0x20: upload and start hotpoint task

Request:

```c
typedef struct{
    uint8_t version;//reserved, kept as 0
    double hp_latitude;//Hotpoint latitude (radian)
    double hp_longitude;//Hotpoint longitude (radian)
    double hp_altitude;//Hotpoint altitude (relative altitude from takeoff point
    double hp_radius;//Hotpoint radius (5m~500m)
    float angle_rate;//Angle rate (0~30°/s)
    uint8_t is_clockwise;// 0->fly in counter-clockwise direction, 1->clockwise direction
    uint8_t start_point_position;//start point position
                                //0: north to the hot point
                                //1: south to the hot point
                                //2: west to the hot point
                                //3: east to the hot point
                                //4: from current position to nearest point on the hot point
    uint8_t yaw_mode;//yaw mode
                    //0: point to velocity direction
                    //1: face inside
                    //2: face ouside
                    //3: controlled by RC
                    //4: same as the starting yaw
    
    uint8_t reserved[11];//reserved

} hotpoint_mission_setting_t;
```

ACK:

```c
struct hotpoint_upload_ack{
    uint8_t ack;
    float max_radius;
};
```

### 0x03, 0x21: stop hotpoint mission

Request:

`uint8_t` with arbitrary value

ACK: `uint8_t`

### 0x03, 0x22: pause hotpoint mission

Request:

```c
uint8_t pause; //0->pause, 1->resume
```

ACK: `uint8_t`

### 0x03, 0x23: set hotpoint idle velocity

```c
typedef struct{
    uint8_t is_clockwise;
    float idle_velocity;
}hotpoint_set_vel_t;
```

ACK: `uint8_t`

### 0x03, 0x24: set hotpoint radius

Request:

```c
float radius;
```

ACK: `uint8_t`

### 0x03, 0x25: reset hotpoint yaw

Requset:

`uint8_t` with arbitrary value

ACK: `uint8_t`

### 0x03, 0x26: download hotpoint task

Request:

`uint8_t` with arbitary value

ACK:

```c
struct hotpoint_download_ack {
    uint8_t ack;
    hotpoint_mission_setting_t hotpoint_task;
};
```

### 0x03, 0x27：enable auto-radiu mode

Request:

```c
struct hotpoint_auto_radiu {
    uint8_t on_off; //1->enable, 0->disable
    int8_t rate;//radiu change rate
};
```

ACK: `uint8_t`

### 0x03, 0x30: upload and start follow me task

Request:

```c
typedef struct{
    uint8_t follow_mode;// follow mode(reserved), set as 0
    uint8_t yaw_mode;//yaw mode
                    //1: point to target
                    //0: controlled by RC
    double init_latitude;//initial position latitude (radian)
    double init_longitude;//initial position longitude (radian)
    uint16_t init_alti;//initial position altitude
    uint16_t init_mag_angle;//reserved
    uint8_t follow_sensitivity;//reserved, set as 0
}follow_me_mission_setting_t;
```

ACK: `uint8_t`

### 0x03, 0x31: stop follow me task

Request:

`uint8_t` with arbitary value

ACK: `uint8_t`

### 0x03, 0x32: pause/resume follow me task

Request:

```c
uint8_t pause; //0->pause, 1->resume
```

ACK: `uint8_t`

### 0x03, 0x33: update target position

Request:
 
```c
typedef struct{
    double latitude; //Target latitude (radian)
    double longitude;//Target longitude (radian)
    uint16_t altitude;//Target altitude
    uint16_t mag_angle;//reserved

}cmd_mission_follow_target_info;
```

NO ACK

---

The following CMD SET/ID are not API but broadcast data protocol, by which developers can check the current mission status and events like how flight data works.

Note: Developers should select the `Ground Station Status` checkbox in DJI Assistant.

    ![](Images/groundstation.png)

### 0x02, 0x03 Current Mission Status Push Information 

There are four kinds of mission status with the same struct size.

Developers can separate them by their first bytes, i.e. `mission_type`.

The 'mission_tpye' is defined with the following enum.
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

waypoint mission push information

```c
typedef struct{
    uint8_t mission_type; //mission type, should be NAVI_MISSION_WAYPOINT
    uint8_t target_waypoint; //current target waypoint index
    uint8_t current_status;//current status
    uint8_t error_notification;//error notification
    uint16_t reserved;//reserved

} cmd_mission_waypoint_status_push_t;
```

hotpoint mission push information

```c
typedef struct{
    uint8_t mission_type;// mission type, should be NAVI_MISSION_HOTPOINT
    uint8_t mission_status;//mission status
                        //0:init
                        //1:running
                        //2:paused
    uint16_t hp_exec_radius; //distance to the hotpoint: cm
    uint8_t reason;
    uint8_t hp_exec_vel;//angular velocity in ground frame, degree * 10
} cmd_mission_hotpoint_status_push_t;
```

follow me mission push information

```c
typedef struct{
    uint8_t mission_type;// mission type, should be NAVI_MISSION_FOLLOWM
    uint8_t reserved_1;
    uint16_t reserved_2;
    uint16_t reserved_3;
}cmd_mission_folowme_status_push_t;
```

the other two status(NAVI_MODE_ATTI & NAVI_MISSION_IOC)

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

### 0x02, 0x04 Mission Event Push Information

There are three kinds of waypoint mission events with the same struct size.

Developers can separate them by their first bytes, i.e. `incident_type`.

The 'incident_type' is defined with the following enum.

~~~c
typedef enum
{
    NAVI_UPLOAD_FINISH,
    NAVI_MISSION_FINISH;
    NAVI_MISSION_WP_REACH_POINT,
}incident_type;
~~~

waypoint mission upload event push information

```c
typedef struct{
    uint8_t incident_type;
    uint8_t is_mission_valid;
    uint16_t estimated_run_time;
    uint16_t reserved;
}cmd_mission_wp_upload_incident_t;
```

waypoint mission finish event push information

```c
typedef struct{
    uint8_t incident_type;
    uint8_t repeat;
    uint16_t reserved_1;
    uint16_t reserved_2;
}cmd_mission_wp_finish_incident_t;
```

waypoint reached event push information

```c
typedef struct{
    uint8_t incident_type;
    uint8_t waypoint_index;
    uint8_t current_status;
    uint8_t reserved_1;
    uint8_t reserved_2;
}cmd_mission_wp_reached_incident_t;
```

## ACK Code
<table>
    <tr>
        <th colspan="4">Common ACK</th>
    </tr>
    <tr>
        <th>0x00</th>
        <th>Success</th>
        <th>0xD0</th>
        <th>mode bar not in F</th>
    </tr>
    <tr>
        <th>0xD1</th>
        <th>not in Navi Mode</th>
        <th>0xD2</th>
        <th>IOC enabled</th>
    </tr>
    <tr>
        <th>0xD3</th>
        <th>mission not init</th>
        <th>0xD4</th>
        <th>missino not running</th>
    </tr>
    <tr>
        <th>0xD5</th>
        <th>mission running</th>
        <th>0xD6</th>
        <th>flight duration not satisfy</th>
    </tr>
    <tr>
        <th>0xD7</th>
        <th>mission with higher priority is running</th>
        <th>0xD8</th>
        <th>GPS health not satisfied</th>
    </tr>
    <tr>
        <th>0xD9</th>
        <th>low battery</th>
        <th>0xDA</th>
        <th>drone not in air when init</th>
    </tr>
    <tr>
        <th>0xDB</th>
        <th>invalid missino parameter</th>
        <th>0xDC</th>
        <th>execution condition not satisfied</th>
    </tr>
    <tr>
        <th>0xDD</th>
        <th>mission will fly through no-fly zone</th>
        <th>0xDE</th>
        <th>HOME point not recorded</th>
    </tr>
    <tr>
        <th>0xDF</th>
        <th>drone in no-fly zone</th>
        <th>0xC0</th>
        <th>altitude higher than max</th>
    </tr>
    <tr>
        <th>0xC1</th>
        <th>altitude lower than min</th>
        <th>0xC7</th>
        <th>too far</th>
    </tr>
    <tr>
        <th>0xC8</th>
        <th>drone not support groundstation functions</th>
        <th>0xC9</th>
        <th>too far from the hotpoint/first waypoint</th>
    </tr>
    <tr>
        <th>0xCA</th>
        <th>drone in beginner mode</th>
        <th>0xF0</th>
        <th>drone is taking off</th>
    </tr>
    <tr>
        <th>0xF1</th>
        <th>drone is landing</th>
        <th>0xF2</th>
        <th>drone is backing to home</th>
    </tr>
    <tr>
        <th>0xF3</th>
        <th>drone is arming</th>
        <th>0xF4</th>
        <th>invalid command</th>
    </tr>
    <tr>
        <th>0xFF</th>
        <th>unknown command/th>
    </tr>
    <tr>
        <th colspan="4">Follow me ACK</th>
    </tr>
    <tr>
        <th>0xB0</th>
        <th>drone too far from mobile</th>
        <th>0xB1</th>
        <th>disconnect time too long</th>
    </tr>
    <tr>
        <th>0xB2</th>
        <th>gimbal pitch too large</th>
    </tr>
    <tr>
        <th colspan="4">Hotpoint ACK</th>
    </tr>
    <tr>
        <th>0xC2</th>
        <th>invalid radius</th>
        <th>0xC3</th>
        <th>velocity too large</th>
    </tr>
    <tr>
        <th>0xC4</th>
        <th>invalid start point</th>
        <th>0xC5</th>
        <th>invalid yaw mode</th>
    </tr>
    <tr>
        <th>0xC6</th>
        <th>too far to back to route</th>
        <th>0xA2</th>
        <th>invalid float number</th>
    </tr>
    <tr>
        <th>0xA3</th>
        <th>invalid laittude/longitude</th>
        <th>0xA6</th>
        <th>invalid direction</th>
    </tr>
    <tr>
        <th>0xA9</th>
        <th>hotpoint already paused</th>
        <th>0xAA</th>
        <th>hotpoint not paused</th>
    </tr>
    <tr>
        <th colspan="4">Waypoint ACK</th>
    </tr>
    <tr>
        <th>0xE0</th>
        <th>invalid mission data</th>
        <th>0xE1</th>
        <th>invalid waypoint data</th>
    </tr>
    <tr>
        <th>0xE2</th>
        <th>planned route too long</th>
        <th>0xE3</th>
        <th>flight route too long</th>
    </tr>
    <tr>
        <th>0xE4</th>
        <th>index larger than max number</th>
        <th>0xE5</th>
        <th>neighbor waypoints too close</th>
    </tr>
    <tr>
        <th>0xE6</th>
        <th>neighbor waypoints too far</th>
        <th>0xE7</th>
        <th>damping checking failed</th>
    </tr>
    <tr>
        <th>0xE8</th>
        <th>invalid action parameter</th>
        <th>0xE9</th>
        <th>waypoint upload not finished</th>
    </tr>
    <tr>
        <th>0xEA</th>
        <th>waypoint task not uploaded</th>
        <th>0xEB</th>
        <th>not all waypoint uploaded</th>
    </tr>
    <tr>
        <th>0xEC</th>
        <th>request is running</th>
        <th>0xED</th>
        <th>cannot pause cause not running</th>
    </tr>
    <tr>
        <th colspan="4">IOC ACK</th>
    </tr>
    <tr>
        <th>0xA0</th>
        <th>too close to HOME point</th>
        <th>0xA1</th>
        <th>IOC type error</th>
    </tr>
    
</table>
