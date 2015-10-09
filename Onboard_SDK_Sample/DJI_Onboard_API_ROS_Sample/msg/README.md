##Notes: 


There are two timestamps you can find in the message declaration.


One is in ROS Header, which is used for ROS MessageFilter and making listeners able to synchronize data received from differnt topics.

The other one is `ts`. It is the hardware timestamp published from N1 flight controller. The `ts` in message is the **exact** time when your drone finds out its attitude/velocity/position without transmission delay(or you can say before transmission delay). They are both necessary for data timestamp align.
