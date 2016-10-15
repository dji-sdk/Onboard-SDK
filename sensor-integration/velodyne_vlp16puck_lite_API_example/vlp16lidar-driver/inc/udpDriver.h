/** @file udpDriver.h
*  @version 3.1.9
*  @date October 7, 2016
*
*  @brief
*  udp and las/pcap file logging header file. 
*
*  @copyright 2016 DJI. All rights reserved.
*
*/

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string>
#include <cstdio>
#include <ctime>

#include <pcap.h>

#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/impl/io_service.hpp>

#include <liblas/liblas.hpp>
#include <eigen3/Eigen/Dense>

#include "vtkPacketFileWriter.h"
#include "vtkVelodyneHDLReader.h"
#include <vtkNew.h>
#include "vtkLASFileWriter.h"

#define portListening 2368  //vlp16
#define BUFF_SIZE     2048


using boost::asio::ip::udp;


class UDPdriver
{
    public:
    UDPdriver(boost::asio::io_service& io_service, unsigned int &loggingTypeFlag, std::string &logFilePathNamePcap, std::string &logFilePathNameLas, unsigned int& printingToTerminal, unsigned int *controlFlag);
    ~UDPdriver();

private:
    void receivePkt(void);
    void handleRec(const boost::system::error_code &error, std::size_t size);
    void dataProcessingAndLogging(unsigned char* dataB, unsigned int dataLength);

    unsigned char recBuffer0[BUFF_SIZE];
    unsigned char recBuffer[BUFF_SIZE];

    udp::socket udpSocket;
	udp::endpoint remoteEndPoint;

    vtkPacketFileWriter  packetFileWriter;
    vtkNew<vtkVelodyneHDLReader> HDLReader;
    vtkLASFileWriter   myVtkLASFileWriter;

    unsigned long pktIndex;
    unsigned int printing2Terminal;

    unsigned int *controlFlagPt;
    unsigned int loggingFileTypeFlag;
    
    unsigned int OnlyOnePacketDebug;
    FILE *debugF, *debugFinput;
    void debugFunc();
    int NumOfPackketsLogged;
    int NumOfPackketsProcessed;
};



