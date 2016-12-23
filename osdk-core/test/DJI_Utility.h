/*
 * DJI_Utility.h
 *
 *  Created on: Dec 13, 2016
 *      Author: oksana
 */

#ifndef OSDK_CORE_TEST_DJI_UTILITY_H_
#define OSDK_CORE_TEST_DJI_UTILITY_H_

#include <chrono>
#include <iostream>

typedef std::chrono::high_resolution_clock high_resolution_clock;
typedef std::chrono::milliseconds milliseconds;
typedef std::chrono::duration<float, std::milli> duration;

class Timer {
 public:
  Timer();
  ~Timer();
  void start();
  void reset();
  duration elapsed() const;

  private:
   high_resolution_clock::time_point _start;
};

#endif /* OSDK_CORE_TEST_DJI_UTILITY_H_ */
