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

#include "stm32f4xx.h"
#include "main.h"

#define sample_flag 0;
#ifdef FLIGHT_CONTROL_SAMPLE
#define sample_flag 1
#elif HOTPOINT_MISSION_SAMPLE
#define sample_flag 2
#elif WAYPOINT_MISSIOM_SAMPLE
#define sample_flag 3
#elif CAMERA_GIMBAL_SAMPLE
#define sample_flag 4
#elif MOBILE_SAMPLE
#define sample_flag 5
#elif TELEMETRY_SAMPLE
#define sample_flag 6
#endif

const int sampleToRun = sample_flag;

/*-----------------------DJI_LIB VARIABLE-----------------------------*/
using namespace DJI::OSDK;

bool           threadSupport = false;
bool           isFrame       = false;
RecvContainer  receivedFrame;
RecvContainer* rFrame  = &receivedFrame;
Vehicle        vehicle = Vehicle(threadSupport);
Vehicle*       v       = &vehicle;

extern TerminalCommand myTerminal;

int
main()
{
  BSPinit();

  delay_nms(30);
  printf("STM32F4Discovery Board initialization finished!\r\n");

  char     func[50];
  uint32_t runOnce = 1;

  while (1)
  {
    // One time automatic activation
    if (runOnce)
    {
      runOnce = 0;

      // Check USART communication
      if (!v->protocolLayer->getDriver()->getDeviceStatus())
      {
        printf("USART communication is not working.\r\n");
        delete (v);
        return -1;
      }

      printf("Sample App for STM32F4Discovery Board:\r\n");
      delay_nms(30);

      printf("\nPrerequisites:\r\n");
      printf("1. Vehicle connected to the Assistant and simulation is ON\r\n");
      printf("2. Battery fully chanrged\r\n");
      printf("3. DJIGO App connected (for the first time run)\r\n");
      printf("4. Gimbal mounted if needed\r\n");
      delay_nms(30);

      //! Initialize functional Vehicle components like
      //! Subscription, Broabcast, Control, Camera, etc
      v->functionalSetUp();
      delay_nms(500);

      // Check if the firmware version is compatible with this OSDK version
      if (v->getFwVersion() > 0 &&
				v->getFwVersion() < extendedVersionBase &&
	      v->getFwVersion() != Version::M100_31)
      {
	      printf("Upgrade firmware using Assistant software!\n");
        delete (v);
        return -1;
      }

      userActivate();
      delay_nms(500);
      /*ACK::ErrorCode ack = waitForACK();
      if(ACK::getError(ack))
      {
        ACK::getErrorCodeMessage(ack, func);
      }*/

      // Verify subscription
      if (v->getFwVersion() != Version::M100_31)
      {
        v->subscribe->verify();
        delay_nms(500);
      }

      // Obtain Control Authority
      v->obtainCtrlAuthority();
      delay_nms(1000);

      switch (sampleToRun)
      {
        case 1:
          printf("\n\nStarting executing position control sample:\r\n");
          delay_nms(1000);
          // Run monitor takeoff
          monitoredTakeOff();
          // Run position control sample
          
	  // For M100 zPosition is 1.2
          float zPosition = 0;
          if(v->getFwVersion() == Version::M100_31)
	  {
	    zPosition = 1.2;
	  }

	  moveByPositionOffset(0, 6, zPosition, 0);
          moveByPositionOffset(6, 0, zPosition, 0);
          moveByPositionOffset(-6, -6, zPosition, 0);
          // Run monitored landing sample
          monitoredLanding();
          break;
        case 2:
          printf("\n\nStarting executing Hotpoint mission sample:\r\n");
          delay_nms(1000);

          // Run Hotpoint mission sample
          runHotpointMission();
          break;
        case 3:
          printf("\n\nStarting executing Waypoint mission sample:\r\n");
          delay_nms(1000);

          // Run Waypoint mission sample
          runWaypointMission();
          break;
        case 4:
          printf("\n\nStarting executing camera gimbal sample:\r\n");
          delay_nms(1000);

          // Run Camera Gimbal sample
          gimbalCameraControl();
          break;
        case 5:
          printf("\n\nStarting executing mobile communication sample:\r\n");
          delay_nms(1000);

          // Run Mobile Communication sample
          v->moc->setFromMSDKCallback(parseFromMobileCallback);
          printf(
            "\n\nMobile callback registered. Trigger command mobile App.\r\n");
          delay_nms(10000);
          break;
        case 6:
          printf("\n\nStarting executing telemetry sample:\r\n");
          delay_nms(1000);

          // Run Telemetry sample
          if (v->getFwVersion() == Version::M100_31)
          {
            getBroadcastData();
          }
          else
          {
            subscribeToData();
          }

          delay_nms(10000);
          break;
        default:
          printf("\n\nPass as preprocessor flag to run desired sample:\r\n");
          printf("FLIGHT_CONTROL_SAMPLE\r\n");
          printf("HOTPOINT_MISSION_SAMPLE\r\n");
          printf("WAYPOINT_MISSION_SAMPLE\r\n");
          printf("CAMERA_GIMBAL_SAMPLE\r\n");
          printf("MOBILE_SAMPLE\r\n");
				  printf("TELEMETRY_SAMPLE\r\n");
          break;
      }
    }
  }
}
