/** @file dji_camera_stream_link.hpp
 *  @version 3.5
 *  @date Dec 2017
 *
 *  @brief The class to read data from the camera

 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJICAMERASTREAMLINK_HH
#define DJICAMERASTREAMLINK_HH
#include "netdb.h"
#include <string>
#include "pthread.h"

#include "dji_camera_image.hpp"

typedef void (*CAMCALLBACK)(void*, uint8_t*, int);

class DJICameraStreamLink
{
public:
  DJICameraStreamLink(CameraType c);
  ~DJICameraStreamLink();
  /* Establish link to camera */
  bool init();

  /* Start the data receiving thread */
  bool start();

  /* Stop the data receiving thread */
  void stop();

  /* do both stop and unInit */
  void cleanup();

  /* start routine for the data receiving thread*/
  static void* readThreadEntry(void *);

  bool isThreadRunning();

  /* register a callback function */
  void registerCallback(CAMCALLBACK f, void* param);

private:
  CameraType  camType;
  std::string camNameStr;
  std::string ip;
  std::string port;
  int fHandle;

  pthread_t readThread;
  int       threadStatus;

  bool isRunning;

  CAMCALLBACK cb;
  void* cbParam;

  /* disconnect link from camera */
  void unInit();

  /* real function to read data from camera */
  void readThreadFunc();
};

#endif // DJICAMERASTREAMLINK_HH
