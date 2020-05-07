/*! @file main.cpp
 *  @version 3.3
 *  @date May 2017
 *
 *  @brief
 *  An exmaple program of DJI-onboard-SDK portable for stm32
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 *
 *******************************************************************************
 *                                                                             *
 *          --------               --------                 --------           *
 *         |        |   USART2    |        |    USART3     |        |          *
 *         |   PC   | <---------> | stm32  |  <----------> |  M100  |          *
 *         |        | (USB-TTL)   |        |               |        |          *
 *         |        |             |        |               |        |          *
 *          --------               --------                 --------           *
 *                                                                             *
 *                                                                             *
 *******************************************************************************
 * */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "stm32f4xx.h"
#include "dji_stm32_helpers.hpp"
#include "osdk_platform.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "usb_bsp.h"
#include "usbh_usr.h"
#include "usbh_core.h"
#ifdef __cplusplus
}
#endif

#define CPU_RATE_DEBUG 0
#define sample_flag 0;
#ifdef FLIGHT_CONTROL_SAMPLE
#define sample_flag 1
#elif HOTPOINT_MISSION_SAMPLE
#define sample_flag 2
#elif WAYPOINT_MISSION_SAMPLE
#define sample_flag 3
#elif CAMERA_GIMBAL_SAMPLE
#define sample_flag 4
#elif MOBILE_SAMPLE
#define sample_flag 5
#elif TELEMETRY_SAMPLE
#define sample_flag 6
#elif TIME_SYNC_CALLBACK_SAMPLE
#define sample_flag 7
#elif TIME_SYNC_POLL_SAMPLE
#define sample_flag 8
#elif PAYLOAD_SAMPLE
#define sample_flag 9
#elif CAMERA_MANAGER_SAMPLE_1
#define sample_flag 10
#elif CAMERA_MANAGER_SAMPLE_2
#define sample_flag 11
#elif GIMBAL_MANAGER_SAMPLE
#define sample_flag 12
#endif

const int sampleToRun = sample_flag;

/*-----------------------DJI_LIB VARIABLE-----------------------------*/
using namespace DJI::OSDK;

bool threadSupport = false;
STM32Setup *env = new STM32Setup();
Vehicle* v = NULL;
T_OsdkTaskHandle mainLoopHandler;
T_OsdkTaskHandle USBProcessHandler;
#if CPU_RATE_DEBUG
T_OsdkTaskHandle debugHandler;
#endif

extern uint64_t timer1Tick;
#if CPU_RATE_DEBUG
void *debugTask(void *p){
  char pcWriteBuffer[512] = {0};
  while (true) {
    delay_nms(3000);
    /*! Debug Only : print the remain free heap size of the OS */
    DSTATUS("xPortGetFreeHeapSize=%d\r\n", xPortGetFreeHeapSize());

    /*! Debug Only : print the info about the task list */
    vTaskList(pcWriteBuffer);
    DSTATUS("Task List Info :\r\n"
            "Task\tState\tPrior\tStack\tNum\r\n"
            "%s\r\n", pcWriteBuffer);

    /*! Debug Only : print the info about the CPU rate */
    memset(pcWriteBuffer, 0, sizeof(pcWriteBuffer));
    vTaskGetRunTimeStats((char *)pcWriteBuffer);
    DSTATUS("CPU Rate Info :\r\n"
            "Task\tTimes\t\tCPU_RATE\r\n"
            "%s\r\n", pcWriteBuffer);
  }
}
#endif
void *USBProcessTask(void *p){
  DSTATUS("Start USB processing ...");
  while(1) {
    USBH_Process(&USB_OTG_Core, &USB_Host);
  }
}
void *mainLoopTask(void *p){
  char func[50];
  uint32_t runOnce = 1;

  /*! init the vehicle */
  env->initVehicle();
  v = env->getVehicle();

  while (1) {
    // One time automatic activation
    if (runOnce) {
      runOnce = 0;

      DSTATUS("Sample App for STM3241G-EVAL Board");
      delay_nms(30);

      DSTATUS("Prerequisites:\n"
      "1. Set flight simulation is ON by DJI Assistant or config tool\n"
      "2. Battery fully chanrged\n"
      "3. App connected (for the first time run)\n"
      "4. Gimbal mounted if needed\n");
      delay_nms(30);

      //! Initialize functional Vehicle components like
      //! Subscription, Broabcast, Control, Camera, etc
      if(v->functionalSetUp()!= 0) {
        DERROR("Unable to initialize some vehicle components! Blocking here !!!");
        while(1) {
          delay_nms(1000);
        }
      }
      delay_nms(500);

      userActivate();
      delay_nms(500);

      if (!v->isM300()) {
        v->setUSBFlightOn(true);
      }

      // Verify subscription
      if (v->getFwVersion() != Version::M100_31) {
        v->subscribe->verify();
        delay_nms(500);
      }

      // Obtain Control Authority
      v->control->obtainCtrlAuthority();
      delay_nms(1000);

      switch (sampleToRun) {
        case 1: {
          DSTATUS("Starting executing position control sample:\r\n");
          delay_nms(1000);
          // Run monitor takeoff
          monitoredTakeOff();
          // Run position control sample

          // For M100 zPosition is 1.2
          float zPosition = 0;
          if (v->getFwVersion() == Version::M100_31) {
            zPosition = 1.2;
          }

          moveByPositionOffset(0, 6, zPosition, 0);
          moveByPositionOffset(6, 0, zPosition, 0);
          moveByPositionOffset(-6, -6, zPosition, 0);
          // Run monitored landing sample
          monitoredLanding();
          break;
        }
        case 2:
          DSTATUS("Starting executing Hotpoint mission sample:\r\n");
          delay_nms(1000);

          // Run Hotpoint mission sample
          runHotpointMission();
          break;
        case 3:
          DSTATUS("Starting executing Waypoint mission sample:\r\n");
          delay_nms(1000);

          // Run Waypoint mission sample
          runWaypointMission();
          break;
        case 4:
          DSTATUS("Starting executing camera gimbal sample:\r\n");
          delay_nms(1000);

          // Run Camera Gimbal sample
          gimbalCameraControl();
          break;
        case 5:
          DSTATUS("Starting executing mobile communication sample:\r\n");
          delay_nms(1000);

          // Run Mobile Communication sample
          v->mobileDevice->setFromMSDKCallback(parseFromMobileCallback);
          DSTATUS(
              "Mobile callback registered. Trigger command mobile "
              "App.\r\n");
          delay_nms(10000);
          break;
        case 6:
          DSTATUS("Starting executing telemetry sample:\r\n");
          delay_nms(1000);

          // Run Telemetry sample
          if (v->getFwVersion() == Version::M100_31) {
            getBroadcastData();
          } else {
            subscribeToData();
          }

          delay_nms(10000);
          break;
        case 7:
          DSTATUS("Starting executing time sync callback sample:\r\n");
          delay_nms(1000);
          time_sync_callback_test();
          delay_nms(1000);
          DSTATUS("test end\r\n");
          break;
        case 8:
          DSTATUS("Starting executing time sync poll sample:\r\n");
          delay_nms(1000);
          time_sync_poll_test();
          delay_nms(1000);
          DSTATUS("test end\r\n");
          break;
        case 9:
          DSTATUS("Starting executing payload communication sample:\r\n");
          delay_nms(1000);

          // Run Payload Communication sample
          v->payloadDevice->setFromPSDKCallback(parseFromPayloadCallback);
          DSTATUS("Payload callback registered.\r\n");
          PayloadSendingTest(30);
          delay_nms(10000);
          break;
        case 10:
          DSTATUS("Starting executing camera manager sample 1:\r\n");
          DSTATUS("Please make sure X5S camera is at the payload 0 site\r\n");
          cameraManagerTest(v, X5S_AT_PAYLOAD_0);
          break;
        case 11:
          DSTATUS("Starting executing camera manager sample 1:\r\n");
          DSTATUS("Please make sure Z30 camera is at the payload 0 site\r\n");
          cameraManagerTest(v, Z30_AT_PAYLOAD_1);
          break;
        case 12:
          DSTATUS("Starting executing gimbal manager sample:\r\n");
          DSTATUS("Please make sure camera is mounted at the payload 0 site\r\n");
          gimbalManagerTest(v, PAYLOAD_INDEX_0);
          break;
        default:
          DSTATUS("Pass as preprocessor flag to run desired sample:\r\n");
          DSTATUS("FLIGHT_CONTROL_SAMPLE\r\n");
          DSTATUS("HOTPOINT_MISSION_SAMPLE\r\n");
          DSTATUS("WAYPOINT_MISSION_SAMPLE\r\n");
          DSTATUS("CAMERA_GIMBAL_SAMPLE\r\n");
          DSTATUS("MOBILE_SAMPLE\r\n");
          DSTATUS("TELEMETRY_SAMPLE\r\n");
          DSTATUS("TIME_SYNC\r\n");
          DSTATUS("PAYLOAD_SAMPLE\r\n");
          DSTATUS("CAMERA_MANAGER_SAMPLE\r\n");
          DSTATUS("GIMBAL_MANAGER_SAMPLE\r\n");
          break;
      }
    }
    delay_nms(10);
  }
}

int main() {
  BSPinit();
    /* Init Host Library */
  USBH_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
  USB_OTG_FS_CORE_ID,
#else
  USB_OTG_HS_CORE_ID,
#endif
  &USB_Host, &CDC_cb, &USR_Callbacks);

  /*! create tasks */
  OsdkOsal_TaskCreate(&mainLoopHandler, mainLoopTask, 1024, NULL);
  OsdkOsal_TaskCreate(&USBProcessHandler, USBProcessTask, 1024, NULL);
#if CPU_RATE_DEBUG
  OsdkOsal_TaskCreate(&debugHandler, debugTask, 1024, NULL);
#endif
  vTaskStartScheduler();
}
