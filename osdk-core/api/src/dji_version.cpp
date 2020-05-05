/*! @file dji_version.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Drone/SDK Version definition for DJI onboardSDK library
 *
 *  @note Since OSDK 3.2.2 (Feb 2017), versioning is handled by the SDK.
 *  You can use the Version::FW macro to target your code towards specific
 * platforms/firmware.
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

#include "dji_version.hpp"

using namespace DJI;
using namespace DJI::OSDK;

const char Version::M100[]   = {0x4D, 0x31, 0x30, 0x30, 0x00};
const char Version::N3[]     = {0x4E, 0x33, 0x00};
const char Version::A3[]     = {0x41, 0x33, 0x00};
const char Version::M210[]   = {0x50, 0x4D, 0x34, 0x31, 0x30, 0x00};
const char Version::M210V2[] = {0x50, 0x4D, 0x34, 0x32, 0x30, 0x00};
const char Version::M300[]   = {0x50, 0x4D, 0x34, 0x33, 0x30, 0x00};
const char Version::M600[]   = {0x50, 0x4D, 0x38, 0x32, 0x30, 0x00};

const Version::FirmWare
Version::FW(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  return (((a << 24) & 0xff000000) | ((b << 16) & 0x00ff0000) |
          ((c << 8) & 0x0000ff00) | (d & 0x000000ff));
}
