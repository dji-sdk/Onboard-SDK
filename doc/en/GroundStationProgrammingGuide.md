#### This documentation is now deprecated, please refer to <https://developer.dji.com/onboard-sdk/documentation/application-development-guides/ground-station-programming-guide.html> in DJI Developer Website.

# Ground Station Programming Guide

This file is a simple roadmap of how to implement groudstation tasks, i.e. the waypoint task, the hotpoint task and the follow me task.

## Waypoint Task

Waypoint is an important subset of groundstation functions.
With the help of waypoint API, developers can make drone fly throught a given group of GPS coordinates.

### How to use

1. Init the Waypoint Task.

  Before starting the waypoint task, developers should upload the coordinates information. There are two parts of uploading in waypoint logic, upload the required information of the whole task at first, then upload every waypoint data within this task. Developers can start the waypoint task if and only if both of the task and they waypoint details are uploaded.
  
  To upload the task information, developers should initialize the following struct:
  
  ```c
  typedef struct WayPointInitData
  {
    uint8_t indexNumber;
    float32_t maxVelocity;
    float32_t idleVelocity;

    uint8_t finishAction;
    uint8_t executiveTimes;
    uint8_t yawMode;
    uint8_t traceMode;
    uint8_t RCLostAction;
    uint8_t gimbalPitch;
    float64_t latitude;
    float64_t longitude;
    float32_t altitude;

    uint8_t reserved[16];
} WayPointInitData;
```

2. Upload Waypoint Data.

  After a succesfful update waypoint task, developers should updload the detail of every waypoint in this task one by one, with its corresponding index.
  Other than the basic parameters of waypoint, it is also possible to set actions when reaching the waypoint. 
  
  ```c
  typedef struct WayPointData
  {
    uint8_t index;

    float64_t latitude;
    float64_t longitude;
    float32_t altitude;
    float32_t damping;

    int16_t yaw;
    int16_t gimbalPitch;
    uint8_t turnMode;

    uint8_t reserved[8];
    uint8_t hasAction;
    uint16_t actionTimeLimit;

    uint8_t actionNumber : 4;
    uint8_t actionRepeat : 4;

    uint8_t commandList[16];//! @note issues here list number is 15
    int16_t commandParameter[16];
  } WayPointData;
  ```
  
  

3. Start Waypoint Task

  Developers can start the waypoint task after the previous two steps.

4. Other Waypoint APIs

  There are severl other waypoint APIs developers can make use of, such as pause, resume and stop the task. Developers can also set and read the idle speed as well as the maximum speed. 
  
  Please refer to the [groundstation documents](GroundStationProtocol.md) for detail.
  
## Hotpoint Task

Hotpoint is a functionality that allows the drone to circle around a given point of interest with certain radius. Developers can 

1. Init and Start Hotpoint Task

  Unlike waypoint task, the drone will execute hotpoint task immediately after the initialization step has been finished.
  
  To initialize the hotpoint task, developers should set the coordinate of point of interest, as well as the radius, velocity and several other parameters, then upload them.
  
  ```c
  typedef struct HotPointData
  {
    uint8_t version;

    float64_t latitude;
    float64_t longitude;
    float64_t height;

    float64_t radius;
    float32_t yawRate; // degree

    uint8_t clockwise;
    uint8_t startPoint;
    uint8_t yawMode;
    uint8_t reserved[11];
  } HotPointData;
  ```

2. Other Hotpoint APIs

  There are severl other hotpoint APIs developers can make use of, such as pause, resume and stop the task. Developers can also set and read the idle speed as well as the radius. 
  
  Please refer to the [groundstation documents](GroundStationProtocol.md) for detail.
  
## Follow Me Task

  Follow me allows your drone to follow the movement of target by itself. However, developers should update the target position and send it to drone in order to achieve the follow me movement.

1. Init and Start Follow Me Task

  Similar to hotpoint, follow me task will start immediately after the initialization step.
  
  To initialize the follow me task, developers should set the target position at first.

  ```c
  typedef struct FollowData
  {
    uint8_t mode;
    uint8_t yaw;
    float64_t latitude;
    float64_t longitude;
    uint16_t height;
    uint16_t angle;
    uint8_t sensitivity;
  } FollowData;
  ```
  
2. Update Target Position

  After starting the follow me task, developers should keep telling the current position of target. i.e. update the position of target. Otherwise the drone will hover in the current position because it believes the target doesn't move.

  ```c
  typedef struct FollowTarget
  {
    float64_t latitude;
    float64_t longitude;
    uint16_t height;
    uint16_t angle;
  } FollowTarget;
  ```
  
3. Other Follow Me APIs

  There are severl other follow me APIs developers can make use of, such as pause, resume and stop the task. 
  
  Please refer to the [groundstation documents](GroundStationProtocol.md) for detail.

## The Status Push Information and Event Push Information

  There is a broadcast of flight data in firmware 2.3. In 3.1, we add a similar protocol to push groundstation status and events.

  Note: Developers should select the `Ground Station Status` checkbox in DJI Assistant in order to receive the pushed info.

  ![](Images/groundstation.png)
  
1. Status Push Information

  The status information takes up CMD SET 0x02 with CMD ID 0x03. 
  
  There are four kinds of status corresponding to the task types, waypoint, hotpoint, follow me and non of above.
  
  Please refer to the [groundstation documents](GroundStationProtocol.md) for detail.

2. Event Push Information

  The event push information takes up CMD SET 0x02 with CMD ID 0x04 and it is specifically designed for waypoint task. 
  
  There are three events included in the push info, data uploaded, task finished and waypoint reached.
  
  Please refer to the [groundstation documents](GroundStationProtocol.md) for detail.
