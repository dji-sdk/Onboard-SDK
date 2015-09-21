## Onboard SDK & M100 FAQ

### 1. Is M100 compatible with DJI GO?
Yes.

### 2. Is M100 compatible with Mobile SDK?  
Yes.

### 3. Are there any no-fly zones for M100?  
Yes, please comply with local regulations. For more, please refer to http://flysafe.dji.com/.

### 4. Can M100 be used for aerial photography?  
Yes. M100 is compatible with third party ‘Gimbal and Camera’ and the DJI Zenmuse X series.

### 5. What does Onboard Device mean?  
In the context of DJI SDK documentation, Onboard Device refers to any devices that can communicate with N1 flight controller and supported by Onboard SDK.

### 6. Which flight platforms are in support of the Onboard SDK?  
For now, only M100.

### 7. Which Onboard Device are supported by M100?  
We will provide ‘Onboard Device Selection Guides for M100’ shortly.

### 8. Is an Onboard Device with an OS running needed in the development of Onboard SDK?  
Not necessarily but preferred. We recommend the combination with Ubuntu and ROS but even a simple STM32 MCU without an OS running on it works.

### 9. What flight control data of M100 can I get via the Onboard SDK?  
Timestamp of the flight controller, Quaternion and Acceleration etc. Please refer to the ‘Onboard SDK API Reference’. https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_API_Doc

### 10. What are the communication ports between an Onboard Device and an N1 flight controller?  
For now, Onboard Device can only communicate with N1 flight controller via the UART port.

### 11. Do M100 support third party video capturing devices? Can I use the M100 built-in ‘Lightbridge’ functionality?  
Yes, M100 support third party video capturing device. If you want to use the M100 built-in ‘Lightbridge’ functionality, all you need is the ‘N1 Video Encoder’.http://store.dji.com/product/n1-video-encoder

### 12. What is the data output frequency of N1?  
The data output frequency of N1 can be set via the N1 assistant software with a range [0, 100Hz].

### 13. Suppose a sensor needs to transmit its sensing data back to the mobile device at a constant frequency. Can this functionality be supported by Onboard SDK?  
Yes. ‘Transparent Transmission’ is designed for this purpose. Please refer to related documentation for more.

### 14. Are there any simulators provided for the development of M100？  
Yes. Please visit https://developer.dji.com/matrice-100/downloads/ for more.

### 15. Can I set the initial height of the ‘take off’ function manually?
For now, no. The initial height is set to be about 1.2 meters.

### 16. What is the recommended transmission rate for N1 flight controller to receive external commands?  
50Hz.

### 17. For M100, what are the built-in wireless data transmission solutions?  
For now, only 2. The ‘Data Transparent Transmission’ and the ‘Lightbridge’.

### 18. For the development of Onboard SDK, can I use some bandwidth from the remote controller to control my own Onboard Device?  
Yes, developers can get the remote control value by Message Package to control Onboard Device.

### 19. How many flight status are included via the Onboard SDK?  
For detail flight status and flight life cycle, please refer to our ‘Onboard API Reference’. https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_API_Doc