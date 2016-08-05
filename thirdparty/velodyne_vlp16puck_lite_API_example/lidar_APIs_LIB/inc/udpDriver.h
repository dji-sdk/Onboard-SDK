
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "vtkPacketFileWriter.h"


#define portListening 2368  //vlp16
#define BUFF_SIZE     2048


using boost::asio::ip::udp;

class UDPdriver
{
    public:
    UDPdriver(boost::asio::io_service& io_service, std::string &logFilePathName, unsigned int& printingToTerminal);
     
private:
    void receivePkt(void);
    void handleRec(const boost::system::error_code &error, std::size_t size);

    unsigned char recBuffer[BUFF_SIZE];
    udp::socket udpSocket;
	udp::endpoint remoteEndPoint;
    vtkPacketFileWriter  packetFileWriter;
    unsigned long pktIndex;
    unsigned int printing2Terminal;

};



