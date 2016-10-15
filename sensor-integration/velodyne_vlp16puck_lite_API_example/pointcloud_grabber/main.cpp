/** @file main.cpp
*  @version 3.1.9
*  @date October 7, 2016
*
*  @brief
*  main file for Velodyne LiDAR vlp-16 LAS/PCAP file logging for DJI onboardSDK
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

#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/impl/io_service.hpp>

#include "wrapper.h"

using namespace std;

boost::mutex io_mutex;
unsigned int controlFlag=0;
boost::condition_variable condition;

unsigned int loggingTypeFlag=2; // 0: pcap only, 1: las file only; 2: both pcap and las files
bool ctrlCOrCloseTerminalFlag = false;

void handler(const boost::system::error_code& error, int signal_number);
void udpAndLoggingProcessing(void);
void managementProcessing(void);


int main(int argc, char *argv[])
{

    int tmpInt;

    if(argc == 2)        {
		tmpInt = atoi(argv[2]);
		if (tmpInt >=0 && tmpInt <=2) loggingTypeFlag=tmpInt;
    }


    boost::lock_guard<boost::mutex> lock(io_mutex);
    boost::thread workerThread1(udpAndLoggingProcessing);
    boost::thread workerThread2(managementProcessing);

    bool tmpFlag = false;

    do
    {
     cout<<"Enter q or Q to quit"<<endl;
     char input;
     cin>>input;
     tmpFlag = (input == 'q' || input == 'Q' ||ctrlCOrCloseTerminalFlag);

    } while (!tmpFlag);

    controlFlag=1;
    condition.notify_one();

    while (controlFlag==1) { };

    workerThread1.interrupt();
    workerThread2.interrupt();

    cout << "Bye!"<<endl;

    return 0;
}


void singalhandler(const boost::system::error_code& error, int signal_number)
{
  if (!error)  {
    ctrlCOrCloseTerminalFlag = true;
  }
}


void udpAndLoggingProcessing(void)
{
    std::string logFilePathName="/home/";
    std::string logFilerFolderNamer="/Vlp16_logfiles/";
    std::string logFileDefaultNamer="Vlp16_log_";
    std::string logFilePcapExtension=".pcap";
    std::string logFileLasExtension=".las";
    std::string logFilePathNamePcap;
    std::string logFilePathNameLas;
    std::string currentUser;

    currentUser = getenv("USER");
    std::cout << "Current User =" << currentUser <<std::endl;

    logFilePathName.append(currentUser);
    logFilePathName.append(logFilerFolderNamer);
    logFilePathName.append(logFileDefaultNamer);

    std::time_t rawTime;
    char tmpBuf[100];

    std::time(&rawTime);
    std::strftime(tmpBuf,100,"%H_%M_%S_%m_%d_%Y",std::localtime(&rawTime));
    std::cout << "Current Local time =";
    std::puts(tmpBuf);

    logFilePathName.append(tmpBuf);
    logFilePathNamePcap=logFilePathName;
    logFilePathNamePcap.append(logFilePcapExtension);
    logFilePathNameLas=logFilePathName;
    logFilePathNameLas.append(logFileLasExtension);

    std::cout << "Current Log file is in =";
    std::cout << logFilePathName <<std::endl;

    unsigned int printingToTerminal=1;   //0: turn it off

    udpAndLogging(loggingTypeFlag, logFilePathNamePcap, logFilePathNameLas, printingToTerminal, &controlFlag);

}

void managementProcessing(void)
{

    boost::asio::io_service io_service2;
    boost::asio::signal_set signals(io_service2, SIGINT, SIGTERM, SIGHUP);

    signals.async_wait(singalhandler);
    io_service2.run();

}


