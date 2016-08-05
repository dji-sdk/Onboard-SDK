// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vtkPacketFileWriter.h"

#ifdef _MSC_VER

#include <windows.h>

namespace {
int gettimeofday(struct timeval * tp, void *)
{
  FILETIME ft;
  ::GetSystemTimeAsFileTime( &ft );
  long long t = (static_cast<long long>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
  t -= 116444736000000000LL;
  t /= 10;  // microseconds
  tp->tv_sec = static_cast<long>( t / 1000000UL);
  tp->tv_usec = static_cast<long>( t % 1000000UL);
  return 0;
}
}

#endif

#include <cstring>

//--------------------------------------------------------------------------------
const unsigned short vtkPacketFileWriter::LidarPacketHeader[21] = {
    0xffff, 0xffff, 0xffff, 0x7660,
    0x0088, 0x0000, 0x0008, 0x0045,
    0xd204, 0x0000, 0x0040, 0x11ff,
    0xaab4, 0xa8c0, 0xc801, 0xffff, // checksum 0xa9b4 //source ip 0xa8c0, 0xc801 is 192.168.1.200
    0xffff, 0x4009, 0x4009, 0xbe04, 0x0000};

//--------------------------------------------------------------------------------
const unsigned short vtkPacketFileWriter::PositionPacketHeader[21] = {
    0xffff, 0xffff, 0xffff, 0x7660,
    0x0088, 0x0000, 0x0008, 0x0045,
    0xd204, 0x0000, 0x0040, 0x11ff,
    0xaab4, 0xa8c0, 0xc801, 0xffff, // checksum 0xa9b4 //source ip 0xa8c0, 0xc801 is 192.168.1.200
    0xffff, 0x7420, 0x7420, 0x0802, 0x0000};

//--------------------------------------------------------------------------------
vtkPacketFileWriter::vtkPacketFileWriter()
{
  this->PCAPFile = 0;
  this->PCAPDump = 0;
}

//--------------------------------------------------------------------------------
vtkPacketFileWriter::~vtkPacketFileWriter()
{
  this->Close();
}

//--------------------------------------------------------------------------------
bool vtkPacketFileWriter::Open(const std::string& filename)
{
  this->PCAPFile = pcap_open_dead(DLT_EN10MB, 65535);
  this->PCAPDump = pcap_dump_open(this->PCAPFile, filename.c_str());

  if (!this->PCAPDump)
    {
    this->LastError = pcap_geterr(this->PCAPFile);
    pcap_close(this->PCAPFile);
    this->PCAPFile = 0;
    return false;
    }

  this->FileName = filename;
  return true;
}

//--------------------------------------------------------------------------------
bool vtkPacketFileWriter::IsOpen()
{
  return (this->PCAPFile != 0);
}

void vtkPacketFileWriter::Close()
{
  if (this->PCAPFile)
    {
    pcap_dump_close(this->PCAPDump);
    pcap_close(this->PCAPFile);
    this->PCAPFile = 0;
    this->PCAPDump = 0;
    this->FileName.clear();
    }
}

//--------------------------------------------------------------------------------
const std::string& vtkPacketFileWriter::GetLastError()
{
  return this->LastError;
}

//--------------------------------------------------------------------------------
const std::string& vtkPacketFileWriter::GetFileName()
{
  return this->FileName;
}

//--------------------------------------------------------------------------------
bool vtkPacketFileWriter::WritePacket(const unsigned char* data, unsigned int dataLength)
  {
    if (!this->PCAPFile)
      {
      return false;
      }

    const unsigned short* headerData;
    struct pcap_pkthdr header;

    std::vector<unsigned char> packetBuffer;
    if (dataLength == 1206)
      {
      headerData = LidarPacketHeader;
      header.caplen = 1248;
      header.len = 1248;
      packetBuffer.resize(1248);
      }
    else if(dataLength == (554 - 42))
      {
      headerData = PositionPacketHeader;
      header.caplen = 554;
      header.len = 554;
      packetBuffer.resize(554);
      }
    else
      {
      return false;
      }

    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    header.ts = currentTime;

    memcpy(&(packetBuffer[0]), headerData, 42);
    memcpy(&(packetBuffer[0]) + 42, data, dataLength);

    pcap_dump((u_char *)this->PCAPDump, &header, &(packetBuffer[0]));
    return true;
  }

//--------------------------------------------------------------------------------
bool vtkPacketFileWriter::WritePacket(pcap_pkthdr* packetHeader, unsigned char* packetData)
  {
    pcap_dump((u_char *)this->PCAPDump, packetHeader, packetData);
    return true;
  }
