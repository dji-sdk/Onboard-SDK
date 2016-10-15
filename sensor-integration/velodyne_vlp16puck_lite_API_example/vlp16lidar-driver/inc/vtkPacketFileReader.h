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
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPacketFileReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPacketFileReader -
// .SECTION Description
//

#ifndef __vtkPacketFileReader_h
#define __vtkPacketFileReader_h

#include <pcap.h>
#include <string>

// Some versions of libpcap do not have PCAP_NETMASK_UNKNOWN
#if !defined(PCAP_NETMASK_UNKNOWN)
  #define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif

class vtkPacketFileReader
{
public:

  vtkPacketFileReader()
  {
    this->PCAPFile = 0;
  }

  ~vtkPacketFileReader()
  {
    this->Close();
  }

  bool Open(const std::string& filename)
  {
    char errbuff[PCAP_ERRBUF_SIZE];
    pcap_t *pcapFile = pcap_open_offline(filename.c_str (), errbuff);
    if (!pcapFile)
      {
      this->LastError = errbuff;
      return false;
      }

    struct bpf_program filter;

    if (pcap_compile(pcapFile, &filter, "udp", 0, PCAP_NETMASK_UNKNOWN) == -1)
      {
      this->LastError = pcap_geterr(pcapFile);
      return false;
      }

    if (pcap_setfilter(pcapFile, &filter) == -1)
      {
      this->LastError = pcap_geterr(pcapFile);
      return false;
      }

    this->FileName = filename;
    this->PCAPFile = pcapFile;
    this->StartTime.tv_sec = this->StartTime.tv_usec = 0;
    return true;
  }

  bool IsOpen()
  {
    return (this->PCAPFile != 0);
  }

  void Close()
  {
    if (this->PCAPFile)
      {
      pcap_close(this->PCAPFile);
      this->PCAPFile = 0;
      this->FileName.clear();
      }
  }

  const std::string& GetLastError()
  {
    return this->LastError;
  }

  const std::string& GetFileName()
  {
    return this->FileName;
  }

  void GetFilePosition(fpos_t* position)
  {
#ifdef _MSC_VER
    pcap_fgetpos(this->PCAPFile, position);
#else
    FILE* f = pcap_file(this->PCAPFile);
    fgetpos(f, position);
#endif
  }

  void SetFilePosition(fpos_t* position)
  {
#ifdef _MSC_VER
    pcap_fsetpos(this->PCAPFile, position);
#else
    FILE* f = pcap_file(this->PCAPFile);
    fsetpos(f, position);
#endif
  }

  bool NextPacket(const unsigned char*& data, unsigned int& dataLength, double& timeSinceStart, pcap_pkthdr** headerReference=NULL)
  {
    if (!this->PCAPFile)
      {
      return false;
      }

    struct pcap_pkthdr *header;
    int returnValue = pcap_next_ex(this->PCAPFile, &header, &data);
    if (returnValue < 0)
      {
      this->Close();
      return false;
      }

    if (headerReference != NULL)
      {
      *headerReference = header;
      dataLength = header->len;
      timeSinceStart = GetElapsedTime(header->ts, this->StartTime);
      return true;
      }

    // The ethernet header is 42 bytes long; unnecessary
    const unsigned int bytesToSkip = 42;
    dataLength = header->len - bytesToSkip;
    data = data + bytesToSkip;
    timeSinceStart = GetElapsedTime(header->ts, this->StartTime);
    return true;
  }

protected:

  double GetElapsedTime(const struct timeval& end, const struct timeval& start)
  {
    return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.00;
  }

  pcap_t* PCAPFile;
  std::string FileName;
  std::string LastError;
  struct timeval StartTime;
};

#endif
