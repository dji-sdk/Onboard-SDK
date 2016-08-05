#include <QCoreApplication>
#include <iostream>
#include <unistd.h>
#include <string>
#include <cstdio>
#include <ctime>
#include <pcap.h>
#include "udpDriver.h"

int main()
{
    std::string logFilePathName="/home/";
    std::string logFilerFolderNamer="/Vlp16_logfiles/";
    std::string logFileDefaultNamer="Vlp16_log_";
    std::string logFileExtension=".pcap";
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
    logFilePathName.append(logFileExtension);

    std::cout << "Current Log file is in =";
    std::cout << logFilePathName <<std::endl;

    unsigned int printingToTerminal=1;   //0: turn it off

    try
    {
        boost::asio::io_service io_service;
        UDPdriver udpDriver(io_service, logFilePathName, printingToTerminal);
		
        io_service.run();

    }
    catch (std::exception& err)
    {
        std::cerr << err.what() << std::endl;
    }
}

