/*! @file mfio_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Multi-Function I/O API usage in a Linux environment.
 *  Shows example usage of the APIs available for controlling the MFIO pins
 *  on the vehicle/FC.
 *
 *  @Copyright (c) 2017 DJI
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

#ifndef DJIOSDK_MFIOSAMPLE_HPP
#define DJIOSDK_MFIOSAMPLE_HPP

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

bool pwmOutputBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool pwmOutputNonBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool gpioLoopbackBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool gpioLoopbackNonBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool adcBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool adcNonBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);

static void getGpiCallBack(DJI::OSDK::Vehicle*      vehicle,
                           DJI::OSDK::RecvContainer recvFrame,
                           DJI::OSDK::UserData      userData);
static void getAdcCallBack(DJI::OSDK::Vehicle*      vehicle,
                           DJI::OSDK::RecvContainer recvFrame,
                           DJI::OSDK::UserData      userData);

#endif // DJIOSDK_MFIOSAMPLE_HPP
