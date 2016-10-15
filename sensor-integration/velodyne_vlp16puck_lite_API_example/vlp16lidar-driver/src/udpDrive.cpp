 /** @file udpDriver.cpp
 *  @version 3.1.9
 *  @date October 7, 2016
 *
 *  @brief
 *  udp and las/pcap file logging file.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */
 
 #include "udpDriver.h"
 
 
 UDPdriver::UDPdriver(boost::asio::io_service& io_service,unsigned int &loggingTypeFlag, std::string &logFilePathNamePcap, std::string &logFilePathNameLas,unsigned int& printingToTerminal, unsigned int *controlFlag)
            :udpSocket(io_service, udp::endpoint(udp::v4(), portListening))
 
 {
     loggingFileTypeFlag=loggingTypeFlag;
     controlFlagPt=controlFlag;
 
     if (loggingFileTypeFlag==0 || loggingFileTypeFlag==2) {
         packetFileWriter.Open(logFilePathNamePcap);
     }
 
     pktIndex=0;
     printing2Terminal=printingToTerminal;
 
 
     std::string correctionFile="VLP-16.xml";
     HDLReader->SetCorrectionsFile(correctionFile);
 
 	if (loggingFileTypeFlag==1 || loggingFileTypeFlag==2) {
         myVtkLASFileWriter.vtkLASFileName(logFilePathNameLas.c_str());
     }
 
     OnlyOnePacketDebug=0;
 
     receivePkt();
 
 };
 
 UDPdriver::~UDPdriver()
 {
     if ( *controlFlagPt==1){
         if (loggingFileTypeFlag==0 || loggingFileTypeFlag==2) packetFileWriter.Close();
         if (loggingFileTypeFlag==1 || loggingFileTypeFlag==2) myVtkLASFileWriter.vtkLASFileClose();
     }
 }
 
 
 void UDPdriver::receivePkt(void)
 {
     // Set option to bind to the same address
     boost::asio::socket_base::reuse_address option(true);
     udpSocket.set_option(option);
     
     udpSocket.async_receive_from(
             boost::asio::buffer(recBuffer0),
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
         unsigned int tmpUsignedInt=numOfBytesTransferred;
 
         if ( *controlFlagPt==0) {
            dataProcessingAndLogging(recBuffer0, tmpUsignedInt);
         }
 
     }   
     else
         throw error;
 
     if ( *controlFlagPt==1) {
         if (loggingFileTypeFlag==0 || loggingFileTypeFlag==2) packetFileWriter.Close();
         if (loggingFileTypeFlag==1 || loggingFileTypeFlag==2) myVtkLASFileWriter.vtkLASFileClose();
 
         *controlFlagPt=2;
     }
 
     boost::this_thread::interruption_point();
 
     receivePkt();
 
 }
 
 
 void UDPdriver::dataProcessingAndLogging(unsigned char* dataB, unsigned int dataLength)
 {
 
     if (loggingFileTypeFlag==0 || loggingFileTypeFlag==2) {
         packetFileWriter.WritePacket(dataB, dataLength);
     }
 
     if (loggingFileTypeFlag==1 || loggingFileTypeFlag==2) {
         HDLReader->ProcessHDLPacket(dataB, dataLength);
 
         const vtkSmartPointer<vtkPolyData> data = HDLReader->GetVtkPoints();
         myVtkLASFileWriter.WriteFrame(data.GetPointer());
     }
 
 	if(printing2Terminal) {
          if((pktIndex%1000) ==0) {
              std::cout << "index=";
              std::cout << pktIndex;
              std::cout << "\n";
              //
              std::cout << "pkt size =";
              std::cout << std::size_t (dataLength);
              std::cout << "\n";
          }
 
     }
 
     pktIndex++;
 
 }

