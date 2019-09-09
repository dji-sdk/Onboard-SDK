/*! @file dji_type.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Data type and Data Structure definitions for use throughout DJI OSDK
 *  @attention Most broadcast data definitions in this file have been
 * deprecated.
 *  See dji_topics.hpp for updated definitions.
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
 */

#ifndef DJI_TYPE
#define DJI_TYPE

#include <cstdio>
#include <stdint.h>

//! Define the UNUSED macro to suppress compiler warnings about unused arguments
#ifdef __GNUC__
#define __UNUSED __attribute__((__unused__))
#define __DELETE(x) delete (char*)x
#else
#define __UNUSED
#define __DELETE(x) delete x

//! @todo fix warning.
#ifndef STM32
#pragma warning(disable : 4100)
#pragma warning(disable : 4800)
#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#pragma warning(disable : 4267)
#pragma warning(disable : 4700)
#pragma warning(disable : 4101)
#endif // STM32
#endif //__GNUC__

#ifdef WIN32
#define __func__ __FUNCTION__
#endif // WIN32

//! @note for ARMCC-5.0 compiler
#ifdef ARMCC
#pragma anon_unions
#endif

#ifdef STM32
typedef unsigned int size_t;
#endif

namespace DJI {
namespace OSDK {

//! This is used as the datatype for all data arguments in callbacks.
typedef void *UserData;

typedef uint64_t time_ms;
typedef uint64_t time_us; // about 0.3 million years

typedef float  float32_t;
typedef double float64_t;

extern char buffer[];

/******************Protocol Related Definitions***************************/

const size_t SESSION_TABLE_NUM = 32;
const size_t CALLBACK_LIST_NUM = 10;

/**
 * @note size is in Bytes
 */
const size_t MAX_INCOMING_DATA_SIZE = 300;
const size_t MAX_ACK_SIZE           = 107;

/**
 * @note some constants for stereo camera
 */
static const uint8_t CAMERA_PAIR_NUM = 5;
static const uint8_t IMAGE_TYPE_NUM  = 10;

//! The Header struct is meant to handle the open protocol header.
typedef struct OpenHeader {
  uint32_t sof : 8;
  uint32_t length : 10;
  uint32_t version : 6;
  uint32_t sessionID : 5;
  uint32_t isAck : 1;
  uint32_t reserved0 : 2; // always 0
  uint32_t padding : 5;
  uint32_t enc : 3;
  uint32_t reserved1 : 24;
  uint32_t sequenceNumber : 16;
  uint32_t crc : 16;
}                    OpenHeader;

typedef struct Command {
  uint16_t sessionMode : 2;
  uint16_t encrypt : 1;
  uint16_t retry : 13;
  uint16_t timeout; // unit is ms
  size_t   length;
  uint8_t *buf;
  uint8_t cmd_set;
  uint8_t cmd_id;
  bool    isCallback;
  int     callbackID;
}                    Command;

typedef struct MMU_Tab {
  uint32_t tabIndex : 8;
  uint32_t usageFlag : 8;
  uint32_t memSize : 16;
  uint8_t *pmem;
} MMU_Tab;

typedef struct CMDSession {
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint8_t *buf;

  uint32_t sessionID : 5;
  uint32_t usageFlag : 1;
  uint32_t sent : 5;
  uint32_t retry : 5;
  uint32_t timeout : 16;
  MMU_Tab *mmu;
  bool     isCallback;
  int      callbackID;
  uint32_t preSeqNum;
  time_ms  preTimestamp;
} CMDSession;

typedef struct ACKSession {
  uint32_t sessionID : 5;
  uint32_t sessionStatus : 2;
  uint32_t res : 25;
  MMU_Tab *mmu;
} ACKSession;

/*!
 * @brief Virtual RC Settings (supported only on Matrice 100)
 */
typedef struct VirtualRCSetting {
  uint8_t enable : 1;
  uint8_t cutoff : 1;
  uint8_t reserved : 6;
} VirtualRCSetting;

/*!
 * @brief Virtual RC data (supported only on Matrice 100)
 */
typedef struct VirtualRCData {
  //! @note this is default mapping data structure for
  //! virtual remote controller.
  //! @todo channel mapping
  uint32_t roll;
  uint32_t pitch;
  uint32_t throttle;
  uint32_t yaw;
  uint32_t gear;
  uint32_t reserved;
  uint32_t mode;
  uint32_t Channel_07;
  uint32_t Channel_08;
  uint32_t Channel_09;
  uint32_t Channel_10;
  uint32_t Channel_11;
  uint32_t Channel_12;
  uint32_t Channel_13;
  uint32_t Channel_14;
  uint32_t Channel_15;
} VirtualRCData;

enum DJI_CAMERA_TAKE_PHOTO_TYPE {
  DJI_CAMERA_TAKE_PHOTO_TYPE_STOP       = 0,
  DJI_CAMERA_TAKE_PHOTO_TYPE_NORMAL     = 1,
  DJI_CAMERA_TAKE_PHOTO_TYPE_HDR        = 2,
  DJI_CAMERA_TAKE_PHOTO_TYPE_BOKEH      = 3,
  DJI_CAMERA_TAKE_PHOTO_TYPE_BURST      = 4,
  DJI_CAMERA_TAKE_PHOTO_TYPE_AEB        = 5,
  DJI_CAMERA_TAKE_PHOTO_TYPE_TIME_LAPSE = 6,
  DJI_CAMERA_TAKE_PHOTO_TYPE_PANO_APP   = 7,
  DJI_CAMERA_TAKE_PHOTO_TYPE_TRACKING   = 8,
  DJI_CAMERA_TAKE_PHOTO_TYPE_RAW_BURST  = 9,
  DJI_CAMERA_TAKE_PHOTO_TYPE_EHDR       = 10,
};

enum DJI_CAMERA_ISO_PARAMETER {
  DJI_CAMERA_ISO_PARAMETER_AUTO            = 0, /*!< auto*/
  DJI_CAMERA_ISO_PARAMETER_AUTO_HIGH_SENSE = 1, /*!< auto (high sense) */
  DJI_CAMERA_ISO_PARAMETER_50              = 2, /*!< 50 */
  DJI_CAMERA_ISO_PARAMETER_100             = 3, /*!< 100 */
  DJI_CAMERA_ISO_PARAMETER_200             = 4, /*!< 200 */
  DJI_CAMERA_ISO_PARAMETER_400             = 5, /*!< 400 */
  DJI_CAMERA_ISO_PARAMETER_800             = 6, /*!< 800 */
  DJI_CAMERA_ISO_PARAMETER_1600            = 7, /*!< 1600 */
  DJI_CAMERA_ISO_PARAMETER_3200            = 8, /*!< 3200 */
  DJI_CAMERA_ISO_PARAMETER_6400            = 9, /*!< 6400 */
  DJI_CAMERA_ISO_PARAMETER_12800           = 10, /*!< 12800 */
  DJI_CAMERA_ISO_PARAMETER_25600           = 11, /*!< 25600 */
  DJI_CAMERA_ISO_PARAMETER_FIXED           = 255, /*!< fixed */
};

enum DJI_CAMERA_RECORDING_TYPE {
  DJI_CAMERA_RECORDING_TYPE_COMMON               = 0,
  DJI_CAMERA_RECORDING_TYPE_DELAY                = 1,
  DJI_CAMERA_RECORDING_TYPE_SLOW_MOTION          = 2,
  DJI_CAMERA_RECORDING_TYPE_QUICK_MOVIE          = 3,
  DJI_CAMERA_RECORDING_TYPE_TIMELAPSE_STATIONARY = 4,
  DJI_CAMERA_RECORDING_TYPE_TIMELAPSE_MOTION     = 5,
  DJI_CAMERA_RECORDING_TYPE_TIMELAPSE_HYPER      = 6,
  DJI_CAMERA_RECORDING_TYPE_FAST_MOTION          = 7,
  DJI_CAMERA_RECORDING_TYPE_EMERGENCY_VIDEO      = 8,
  DJI_CAMERA_RECORDING_TYPE_HYPERLAPSE           = 9,
  DJI_CAMERA_RECORDING_TYPE_MARK_VIDEO           = 10,
};

enum DJI_CAMERA_RECORDING_CONTROL {
  DJI_CAMERA_RECORDING_CONTROL_STOP   = 0,
  DJI_CAMERA_RECORDING_CONTROL_BEGIN  = 1,
  DJI_CAMERA_RECORDING_CONTROL_PAUSE  = 2,
  DJI_CAMERA_RECORDING_CONTROL_RESUME = 3,
};


} // namespace OSDK
} // namespace DJI

#endif // DJI_TYPE
