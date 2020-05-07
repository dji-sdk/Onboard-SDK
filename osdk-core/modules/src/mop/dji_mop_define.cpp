/** @file dji_mop_define.cpp
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


#include "dji_mop_define.hpp"
#include "mop.h"
#include "mop_entry_osdk.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::MOP;

MopErrCode MOP::getMopErrCode(int ret) {
  switch (ret) {
    case MOP_SUCCESS : return MOP_PASSED;
    case MOP_ERR_FAILED : return MOP_FAILED;
    case MOP_ERR_CRC : return MOP_CRC;
    case MOP_ERR_PARM : return MOP_PARM;
    case MOP_ERR_NOMEM : return MOP_NOMEM;
    case MOP_ERR_NOTREADY : return MOP_NOTREADY;
    case MOP_ERR_SEND : return MOP_SEND;
    case MOP_ERR_RECV : return MOP_RECV;
    case MOP_ERR_TIMEOUT : return MOP_TIMEOUT;
    case MOP_ERR_RESBUSY : return MOP_RESBUSY;
    case MOP_ERR_RESOCCUPIED : return MOP_RESOCCUPIED;
    case MOP_ERR_CONNECTIONCLOSE : return MOP_CONNECTIONCLOSE;
    case MOP_ERR_NOTIMPLEMENT : return MOP_NOTIMPLEMENT;
    default : return MOP_UNKNOWN_ERR;
  }
}
