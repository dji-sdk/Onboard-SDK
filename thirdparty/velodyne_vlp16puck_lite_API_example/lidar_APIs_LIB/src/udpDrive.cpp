#include "udpDriver.h"


UDPdriver::UDPdriver(boost::asio::io_service& io_service,std::string &logFilePathName,unsigned int& printingToTerminal)
           :udpSocket(io_service, udp::endpoint(udp::v4(), portListening))

{
    packetFileWriter.Open(logFilePathName);
    pktIndex=0;
    printing2Terminal=printingToTerminal;

    receivePkt();
};



void UDPdriver::receivePkt(void)
{
    udpSocket.async_receive_from(
            boost::asio::buffer(recBuffer),
            remoteEndPoint,
            boost::bind(
                &UDPdriver::handleRec,
                this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
}

void UDPdriver::handleRec(const boost::system::error_code &error, std::size_t size)
{
    size_t numOfBytesTransferred=size;

    if (!error && (numOfBytesTransferred > 0))
    {
        pktIndex++;

        if(printing2Terminal) {
            if((pktIndex%1000) ==0) {
                //std::cout << "pkt[" << std::string(&recBuffer[0], &recBuffer[0]+numOfBytesTransferred) << "]\n";
                //
                std::cout << "index=";
                std::cout << pktIndex;
                std::cout << "\n";

                //
                std::cout << "pkt size =";
                std::cout << std::size_t (numOfBytesTransferred);
                std::cout << "\n";
            }

            /*
            std::cout << "\npkt [";
            for (size_t i = 0; i<numOfBytesTransferred; i++) printf("%02x", recBuffer[i]);
            std::cout << "]\n\n";
            */
            //
       }

       unsigned int tmpUsignedInt=numOfBytesTransferred;
       packetFileWriter.WritePacket(recBuffer, tmpUsignedInt);
    }
    else
        throw error;

    receivePkt();
}
