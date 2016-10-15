/** @file wrapper.cpp
*  @version 3.1.9
*  @date October 7, 2016
*
*  @brief
*  wrapper file of udp and las/pcap file logging lib.
*
*  @copyright 2016 DJI. All rights reserved.
*
*/

#include "udpDriver.h"
#include "wrapper.h"

using namespace std;

void udpAndLogging(unsigned int &loggingTypeFlag, std::string &logFilePathNamePcap, std::string &logFilePathNameLas, unsigned int& printingToTerminal, unsigned int *controlFlag)
{
    try
    {
        boost::asio::io_service io_service1;
        UDPdriver udpDriver(io_service1,loggingTypeFlag,logFilePathNamePcap, logFilePathNameLas,printingToTerminal,controlFlag);
        io_service1.run();

    }
    catch (std::exception& err)
    {
        std::cerr << err.what() << std::endl;
    }

}

