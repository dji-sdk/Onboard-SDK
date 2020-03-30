/** @file dji_hms_internal.hpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief HMS((Health Management System) error code table and some basic func for HMS API
 *
 *  @Copyright (c) 2019 DJI
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

#ifndef ONBOARDSDK_DJI_HMS_INTERNAL_HPP
#define ONBOARDSDK_DJI_HMS_INTERNAL_HPP
#include "dji_type.hpp"
#include <iostream>
using namespace std;

namespace DJI{
namespace OSDK{

/*! the type of HMS's error code information*/
typedef struct HMSErrCodeInfo {
    uint32_t alarmId;            /*! error code*/
    std::string groundAlarmInfo; /*! alarm information when the flight is on the ground*/
    std::string flyAlarmInfo;    /*! alarm information when the flight is in the air*/
} HMSErrCodeInfo;

/*! the length of HMS's error code table*/
const uint32_t dbHMSErrNum = 600;

extern void encodeSender(const uint8_t sender,uint8_t & deviceType, uint8_t & deviceIndex);
extern bool replaceStr(string &str, const string oldReplaceStr, const string newReplaceStr);
 }
  }
#endif //ONBOARDSDK_DJI_HMS_INTERNAL_HPP