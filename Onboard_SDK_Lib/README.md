###DJI Onboard SDK Library


This is an officially released DJI Onboard SDK libaray, which you can include and use it directly.

####Feature
 
1. Provided namespace `DJI::onboardSDK`.
 
2. Provided class interface `DJI::onboardSDK::API`.

3. Replaced all static and global variables in a more portable way.

4. Reorganized multi-thread functions into two threads, sending and receivng.
 
5. Provided hardware interfaces `DJI::onboardSDK::HardDriver` for a better compatibility. We have tested the `MinGW` and `QT` platform`, `MSVC` and `ARMCC` will be coming soon. 
 
6. Supported group-control ability for multiple Matrice 100. (coming soon)
 

####How to use
0. Modify the `typedef` of *thread mutex* and *serial port* in `DJI_Config.h` 

1. Inheritance `HardDriver` class. Impllement the `lock` and `unlock` functions, together with several other virtual functions.  

2. Create a new API sample with your inheritanced class working as the pointer of hardware interface.

3. Create two life-long threads(or one, at least) running `sendPoll` and `readPoll` functions repeatedly. (**Note**: the frequency should be lower 100Hz)


---

Note:

Currently the Cmdline/ROS/QT sample programs are still running with the old version DJI Onboard SDK library. The updating procedures are processed step by step, therefore all sample programs will be updated around the corner.