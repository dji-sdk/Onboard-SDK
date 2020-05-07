/** @file dji_mop_define.hpp
 *  @version 4.0
 *  @date Jan 2020
 *
 *  @brief Enumeration of all mop data types, structures.
 *
 *  @Copyright (c) 2020 DJI
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

#ifndef DJI_MOP_DEFINE_HPP
#define DJI_MOP_DEFINE_HPP

#include "mop_entry_osdk.h"
#include "dji_log.hpp"

/*!
 * Top-level namespace
 */
namespace DJI {
/*!
 * Onboard SDK related commands
 */
namespace OSDK {
/*! @brief This namespace encapsulates all available telemetry topics through
 * either
 * Broadcast or Subscribe
 */
namespace MOP {

typedef enum MopErrCode {
  MOP_PASSED,
  MOP_FAILED,
  MOP_CRC,
  MOP_PARM,
  MOP_NOMEM,
  MOP_NOTREADY,
  MOP_SEND,
  MOP_RECV,
  MOP_TIMEOUT,
  MOP_RESBUSY,
  MOP_RESOCCUPIED,
  MOP_CONNECTIONCLOSE,
  MOP_NOTIMPLEMENT,
  MOP_UNKNOWN_ERR,
} MopErrCode;

typedef enum {
  RELIABLE,
  UNRELIABLE,
} PipelineType;

typedef enum SlotType {
  SLOT_0 = 0,
  SLOT_1 = 1,
  SLOT_2 = 2,
} SlotType;

/*! User define the pipeline in the range of uint16_t */
typedef uint16_t PipelineID;

MopErrCode getMopErrCode(int ret);

}
}
}
#endif // DJI_MOP_DEFINE_HPP
