/*
 * DJI_Utility.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: oksana
 */

#include "DJI_Utility.h"

Timer::Timer(){
  reset();
}

Timer::~Timer(){}

void Timer::start() {
  _start = high_resolution_clock::now();
}

void Timer::reset() {
  _start = high_resolution_clock::now();
}

duration Timer::elapsed() const {
  return std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start);
}
