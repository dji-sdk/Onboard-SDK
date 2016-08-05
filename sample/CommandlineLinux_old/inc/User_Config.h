/*! @file User_Config.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  User-side configuration for the Linux Onboard SDK interface. 
 *  Meant to provide configuration capability outside the core library
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define API_DEBUG_DATA

//! If using Manifold, the UART2 port shows up as /dev/ttyTHS1. 
//! The Expansion I/O port shows up as /dev/ttyTHS0. 
//! If using a USB-Serial adapter, use /dev/ttyUSB0 
//! (may differ if you have other USB-Serial devices plugged in)
std::string deviceName = "/dev/ttyUSB0";
//! The keyLocation assumes you are running the sample from within the bin folder.
std::string keyLocation = "../key.txt";
//! Default baudRate is 230400. If you want to change this value, also go to 
//! DJI Assistant 2's SDK tab and change it to match this value.
unsigned int baudRate = 230400;
//! Available choices for targetVersion are versionM100_23, versionA3_31 and versionM100_31
Version targetVersion = versionM100_31; 


#endif //USER_CONFIG_H
