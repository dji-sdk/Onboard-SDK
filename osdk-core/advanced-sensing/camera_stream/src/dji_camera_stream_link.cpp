/*
 * DJI Onboard SDK Advanced Sensing APIs
 *
 * Copyright (c) 2017-2018 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 * @file dji_camera_stream_link.cpp
 *  @version 3.5
 *  @date Dec 2017
 *
 */

#include "dji_camera_stream_link.hpp"
#include "dji_log.hpp"

#ifndef WIN32
  #include <unistd.h>
  #include <cstdlib>
  #include <cstring>
#else
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <wspiapi.h>
#endif

#include "udt.h"

using namespace UDT;

void* monitor(void* s);

#define UDT_SERVER_IP	    "192.168.42.2"
#define UDT_SERVER_PORT_MAIN 	"40001"
#define UDT_SERVER_PORT_FPV  	"40003"
#define RECEIVE_SIZE   128000

// Helper function to free the addresses
void freeAddresses(struct addrinfo *local, struct addrinfo *peer)
{
  freeaddrinfo(local);
  freeaddrinfo(peer);
}

DJICameraStreamLink::DJICameraStreamLink(CameraType c)
  : camType(c),
    ip(std::string(UDT_SERVER_IP)),
    fHandle(-1),
    threadStatus(-1),
    isRunning(false),
    cb(NULL),
    cbParam(NULL)
{
  camNameStr = ((c==FPV_CAMERA) ? std::string("FPV_CAMERA") : std::string("MAIN_CAMERA"));
  port = ((c==FPV_CAMERA) ? std::string(UDT_SERVER_PORT_FPV) : std::string(UDT_SERVER_PORT_MAIN));
}

DJICameraStreamLink::~DJICameraStreamLink()
{
  cleanup();
}

bool DJICameraStreamLink::init()
{
  UDT::startup();

  fHandle = -1;

  struct addrinfo *local = nullptr, *peer = nullptr;
  struct addrinfo hints;

  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_flags = AI_PASSIVE;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  if (0 != getaddrinfo(NULL, port.c_str(), &hints, &local))
  {
    DDEBUG_PRIVATE("First try, incorrect network address.");
    freeAddresses(local, peer);
    return false;
  }

  if (0 != getaddrinfo(ip.c_str(), port.c_str(), &hints, &peer))
  {
    DDEBUG_PRIVATE( "Incorrect server address.");
    freeAddresses(local, peer);
    return false;
  }

//#ifdef DO_DEBUG
//  cout << "Got valid server address. " << endl;
//  cout << hints.ai_family <<" " << hints.ai_socktype <<" "<< hints.ai_protocol<< endl;
//  cout << local->ai_family <<" " << local->ai_socktype <<" "<< local->ai_protocol<< endl;
//  cout << peer->ai_family <<" " << peer->ai_socktype <<" "<< peer->ai_protocol<< endl;
//#endif

  static int optval = 100;

  if(fHandle == -1)
  {
    fHandle = UDT::socket(local->ai_family, local->ai_socktype, local->ai_protocol);
    //UDT_RCVTIMEO 	int 	Receiving call timeout (milliseconds). 	Default -1 (infinite).
    UDT::setsockopt(fHandle, 0, UDT_RCVTIMEO, &optval, sizeof(int));
  }

  UDTSTATUS status = UDT::getsockstate(fHandle);
  if(status == INIT || status == OPENED)
  {
    if (UDT::ERROR == UDT::connect(fHandle, peer->ai_addr, peer->ai_addrlen))
    {
      DERROR_PRIVATE("Connect to %s failed, Error: %s\n", camNameStr.c_str(),
             UDT::getlasterror().getErrorMessage());
      freeAddresses(local, peer);
      UDT::close(fHandle);
      return false;
    }
  }
  else if(status != CONNECTED)
  {
    freeAddresses(local, peer);
    unInit();
    return false;
  }

  freeAddresses(local, peer);
  DSTATUS_PRIVATE("Connect to %s successful\n", camNameStr.c_str());
  //cout << "init successful" << endl;
  return true;
}

void DJICameraStreamLink::unInit()
{
  if(-1 !=fHandle)
  {
    UDT::close(fHandle);
    fHandle = -1;
  }
}

bool DJICameraStreamLink::start()
{
  if (isRunning)
  {
    DSTATUS_PRIVATE("Already reading data from %s\n", camNameStr.c_str());
    return false;
  }

  threadStatus = pthread_create(&readThread, NULL, DJICameraStreamLink::readThreadEntry, this);
  if (threadStatus != 0)
  {
    DERROR_PRIVATE("Error creating camera reading thread for %s\n", camNameStr.c_str());
    DERROR_PRIVATE("pthread_create returns %d\n", threadStatus);
    return false;
  }
  else
  {
    isRunning = true;
    return true;
  }
}

void DJICameraStreamLink::stop()
{
  isRunning = false;
  if(0 == threadStatus)
  {
    pthread_join(readThread, NULL);
    threadStatus = -1;
  }
}

void DJICameraStreamLink::cleanup()
{
  stop();
  unInit();
}

void* DJICameraStreamLink::readThreadEntry(void * c)
{
  (reinterpret_cast<DJICameraStreamLink*>(c))->readThreadFunc();
  return NULL;
}

void DJICameraStreamLink::readThreadFunc()
{
  DSTATUS_PRIVATE("**** %s data reading thread start! ****\n", camNameStr.c_str());

  int retryConnect = 0;
  int retryReading = 0;

//  while(!init() && isRunning)
//  {
//#ifdef DO_DEBUG
//    cout << "Try to connect to " << ip << ":" << port << " for " << retryCount << "time(s)" << endl;
//#endif
//    if(10 == retryCount++)
//    {
//      isRunning = false;
//      unInit();
//      cout << "Unable to connect to camera, quit thread ..." << endl;
//      return;
//    }
//  }

  while (isRunning)
  {
    int rcvLen=0;
    char rcvBuffer[RECEIVE_SIZE];
    if (UDT::ERROR != (rcvLen = UDT::recv(fHandle, rcvBuffer, RECEIVE_SIZE, 0)))
    {
      retryReading = 0;
      if(rcvLen)
      {
        //vector<uint8_t> temp(rcvBuffer, rcvBuffer+rcvLen);
        //cout << "push to buffer: " << rcvLen << " bytes... ";
        //streamBufferQueue.push(temp);
        //cout << "done!" << endl;
        if(cb)
        {
          (*cb)(cbParam, reinterpret_cast<uint8_t *>(&rcvBuffer[0]), rcvLen);
        }
      }
      else
      {
        DDEBUG_PRIVATE("Reading length 0\n");
      }
    }
    else if ((retryReading++) > 10)
    {
      DSTATUS_PRIVATE("Unable to read from %s lost, retry connecting ...\n", camNameStr.c_str());

      retryConnect = 0;
      while(!init() && isRunning)
      {
        usleep(1e5);
        if(10 == retryConnect++)
        {
          isRunning = false;
          unInit();
          DERROR_PRIVATE("Unable to reconnect to %s ..., quit reading thread\n", camNameStr.c_str());
          return;
        }
      }
    }
    usleep(2e4); //50 Hz
  }

  unInit();
  DSTATUS_PRIVATE("**** %s reading thread stopped\n", camNameStr.c_str());
}

void DJICameraStreamLink::registerCallback(CAMCALLBACK f, void* param)
{
  cb = f;
  cbParam = param;
}

bool DJICameraStreamLink::isThreadRunning()
{
  return isRunning;
}
