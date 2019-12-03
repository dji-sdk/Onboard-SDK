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


} // namespace OSDK
} // namespace DJI

#endif // DJI_TYPE
