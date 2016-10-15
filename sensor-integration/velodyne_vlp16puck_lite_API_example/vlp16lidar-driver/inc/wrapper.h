/** @file wrapper.h
*  @version 3.1.9
*  @date October 7, 2016
*
*  @brief
*  header file for wrapper file of udp and las/pcap file logging lib.
*
*  @copyright 2016 DJI. All rights reserved.
*
*/

#ifndef WRAPPER_H
#define WRAPPER_H


void udpAndLogging(unsigned int &loggingTypeFlag, std::string &logFilePathNamePcap, std::string &logFilePathNameLas, unsigned int& printingToTerminal, unsigned int *controlFlag);


#endif // WRAPPER_H
