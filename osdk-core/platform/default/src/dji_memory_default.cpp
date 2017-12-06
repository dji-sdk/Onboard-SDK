/*! @file dji_memory_default.cpp
 *  @version 3.4
 *  @date Dec 2017
 *
 *  @brief Data protection and thread management abstract classes.
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


#include "dji_memory_default.hpp"

#include "dji_log.hpp"

#ifdef qt
#elif defined(stm32)
#elif defined(__linux__)
#include <pthread.h>

namespace DJI
{
namespace OSDK
{
class MutexPrivate
{
public:
  MutexPrivate()
  {
    _mutex = PTHREAD_MUTEX_INITIALIZER;
  }
  ~MutexPrivate()
  {
    pthread_mutex_destroy(&_mutex);
  }

public:
  inline void lock()
  {
    pthread_mutex_lock(&_mutex);
  }
  inline void unlock()
  {
    pthread_mutex_unlock(&_mutex);
  }
  pthread_mutex_t _mutex;
}; // class DJI::OSDK::MutexPrivate

} // namespace OSDK
} // namespace DJI
#else // default non-functional driver

namespace DJI
{
namespace OSDK
{

class MutexPrivate
{
public:
  MutexPrivate()
  {
    //! @note cannot use DLOG here
  }
  ~MutexPrivate()
  {
  }

public:
  inline void lock()
  {
  }
  inline void unlock()
  {
  }
}; // class MutexPrivate

} // namespace OSDK
} // namespace DJI

#endif // ugly cross platform support

using namespace DJI::OSDK;

MutexDefault::MutexDefault()
  : instance(new MutexPrivate())
{
  //! @note cannot use DLOG here
}

MutexDefault::~MutexDefault()
{
  delete instance;
}

void
MutexDefault::lock()
{
  instance->lock();
}

void
MutexDefault::unlock()
{
  instance->unlock();
}
