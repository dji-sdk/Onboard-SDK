##DJI Onboard SDK Library


This is an officially released DJI Onboard SDK libaray, which you can include and use directly.

####Feature
 
1. Provide namespace `DJI::onboardSDK`.
 
2. Provide class interface `DJI::onboardSDK::API`.

3. Replace all static and global variables in a more portable way.

4. Reorganize multi-thread functions into two threads, sending and receivng.
 
5. Provide hardware interfaces `DJI::onboardSDK::HardDriver` for a better compatibility. We have tested the `MinGW`, `QT` platform`, `MSVC` and `ARMCC`. 
 

####How to use

1. Inheritance `HardDriver` class. Impllement the `lock` and `unlock` functions, together with several other virtual functions.  

2. Create a new API sample with your inheritanced class working as the pointer of hardware interface.

3. Create two life-long threads(or one, at least) running `sendPoll` and `readPoll` functions repeatedly. (**Note**: the frequency should be lower 100Hz)

Please read the code in `include` and `source` for more detail.


---

##DJI SDK 官方库文件

####特性：
1. 启用命名空间（namespace）DJI和，命名空间DJI::onboardSDK
 
2. 提供了名为DJI::onboardSDK::API的接口函数类。
         用于以后提供多机协同支持，
收纳了所有的静态分配变量，提高代码可移植性
 
 
3. 压缩了旧接口的多线程内容，将线程数压缩成为发送线程和接收线程两个函数
 
4. 抽象硬件接口函数类HardDriver（全称DJI::onboardSDK::HardDriver），用以提供全平台兼容性。
    
    包括但不限于：

        MinGW 
        QT 
        MSVC
        ARMCC
 
####具体使用方法：

1. 将HardDriver类继承，实现其虚函数表中的全部函数以及线程锁的lock及unlock函数。

2. 创建一个新的API实例，传入刚才继承的HardDriver实例作为硬件接口。

3. 创建两个线程（或者一个），在while(1)函数体中枚举API实例中的sendPoll 和readPoll接口。
 
