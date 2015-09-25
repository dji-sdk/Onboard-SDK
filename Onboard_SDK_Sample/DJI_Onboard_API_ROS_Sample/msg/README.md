##Notes: 


There are two timestamps you can find in the message declaration.


One is in ROS Header, which is used for ROS MessageFilter and making listeners able to synchronize data received from differnt topics.

The other one is `ts`. It is the hardware timestamp published from N1 flight controller. Publishing `ts` is a feature asked by @xuhao1. The `ts` in message is the **exact** time when your drone finds out its attitude/velocity/position without transmission delay(or you can say before transmission delay). As @xuhao1 told me, `ts` is quite necessary when you're trying to fuse sensor data from Guidance, cause they share the same timestamp from N1.
