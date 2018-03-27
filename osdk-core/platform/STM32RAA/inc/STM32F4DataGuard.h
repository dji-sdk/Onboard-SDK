

#ifndef STM32F4DATAGUARD_H
#define STM32F4DATAGUARD_H

#include "dji_thread_manager.hpp"

namespace DJI
{
namespace OSDK
{

/*! @brief Data Protection
 *
 */
class STM32F4DataGuard : public DJI::OSDK::ThreadAbstract
{
public:
  STM32F4DataGuard();
  ~STM32F4DataGuard();

  void init();

public:
  void lockRecvContainer();
  void freeRecvContainer();

  void lockMSG();
  void freeMSG();

  void lockACK();
  void freeACK();

  void lockProtocolHeader();
  void freeProtocolHeader();

  void lockNonBlockCBAck();
  void freeNonBlockCBAck();

  void lockStopCond();
  void freeStopCond();

  void lockFrame();
  void freeFrame();

  void notify();
  void notifyNonBlockCBAckRecv();
  void wait(int timeoutInSeconds);
  void nonBlockWait();
};

} // namespace OSDK
} // namespace DJI

#endif // STM32F4DATAGUARD_H
