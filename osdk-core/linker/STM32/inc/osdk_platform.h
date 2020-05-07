/**
 ********************************************************************
 * @file    osdk_platform.h
 * @version V2.0.0
 * @date    2019/8/30
 * @brief   This is the header file for define OSDK platform interfaces.
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OSDK_PLATFORM_H
#define OSDK_PLATFORM_H

/* Includes ------------------------------------------------------------------*/
#ifdef __linux__
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif
#include "osdk_typedef.h"

#define OSDK_TASK_STACK_SIZE_DEFAULT 1024

#ifdef __cplusplus
extern "C" {
#endif

//#define OS_DEBUG

/* Exported constants --------------------------------------------------------*/
/**
* @brief Platform handle of thread task operation.
*/
typedef void *T_OsdkTaskHandle;
/**
* @brief Platform handle of mutex operation.
*/
typedef void *T_OsdkMutexHandle;
/**
* @brief Platform handle of semaphore operation.
*/
typedef void *T_OsdkSemHandle;

typedef struct {
    int fd;
}T_UartObj;

#ifdef __linux__
typedef struct {
    void* handle;
    uint16_t epIn;
    uint16_t epOut;
}T_UsbBulkObj;
#endif

typedef struct {
    union {
        T_UartObj uartObject;
#ifdef __linux__
        T_UsbBulkObj bulkObject;
#endif
    };
}T_HalObj;

/* Exported types ------------------------------------------------------------*/
typedef struct {
    /*! Specifies uart init interface that need register by this format. Users need to implement the initialization
     * interface of their serial device and register it. For the related parameter settings of the serial port, please
     * refer to the Quick Start document.*/
    E_OsdkStat (*UartInit)(const char *port, const int baudrate, T_HalObj *obj);

    /*! Specifies uart write data interface, need register by this format. Users need to implement the sending
     * interface of their serial device and register it. Please use the relevant tools to test the interface before
     * registering. */
    E_OsdkStat (*UartWriteData)(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen);

    /*! Specifies uart read data interface that need register by this format. Users need to implement the read
     * interface of their serial device and register it. Please use the relevant tools to test the interface before
     * registering.*/
    E_OsdkStat (*UartReadData)(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen);

    /*! Specifies uart close interface that need register by this format. Users need to implement the close
     * interface of their serial device and register it. Please use the relevant tools to test the interface before
     * registering.*/
    E_OsdkStat (*UartClose)(T_HalObj *obj);
} T_OsdkHalUartHandler;

#ifdef __linux__
typedef struct {
  /*! Specifies usb bulk init interface that need register by this format. Users need to implement the initialization
   * interface of their usb device and register it. For the related parameter settings of the usb port, please
   * refer to the Quick Start document.*/
  E_OsdkStat (*USBBulkInit)(uint16_t pid, uint16_t vid, uint16_t num, uint16_t epIn, uint16_t epOut, T_HalObj *obj);

  /*! Specifies usb bulk write data interface, need register by this format. Users need to implement the sending
   * interface of their usb device and register it. Please use the relevant tools to test the interface before
   * registering. */
  E_OsdkStat (*USBBulkWriteData)(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen);

  /*! Specifies usb bulk read data interface that need register by this format. Users need to implement the read
   * interface of their usb device and register it. Please use the relevant tools to test the interface before
   * registering.*/
  E_OsdkStat (*USBBulkReadData)(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen);

  /*! Specifies usb bulk close interface that need register by this format. Users need to implement the close
   * interface of their usb device and register it. Please use the relevant tools to test the interface before
   * registering.*/
  E_OsdkStat (*USBBulkClose)(T_HalObj *obj);
} T_OsdkHalUSBBulkHandler;
#endif

typedef struct {
    /*! Specifies payload create task interface that need register by this format. This interface is mainly used to create
     * task. Users need to adapt according to their own platform and system. Also need test the implemented interface
     * before registration to ensure that the registration can be used normally. For details of the parameters, please
     * refer to the following API interface #OsdkOsal_TaskCreate. */
    E_OsdkStat (*TaskCreate)(T_OsdkTaskHandle *task, void *(*taskFunc)(void *), uint32_t stackSize, void *arg);

    /*! Specifies payload destroy the created task interface that need register by this format. This interface is mainly
     * used to destroy the created task. Users need to adapt according to their own platform and system. Also need test
     * the implemented interface before registration to ensure that the registration can be used normally. For details of
     * the parameters, please refer to the following API interface #OsdkOsal_TaskDestroy. */
    E_OsdkStat (*TaskDestroy)(T_OsdkTaskHandle task);

    /*! Specifies payload task sleep interface that need register by this format. This interface is mainly used to let
     * task sleep. Users need to adapt according to their own platform and system. Also need test the implemented interface
     * before registration to ensure that the registration can be used normally. For details of the parameters, please
     * refer to the following API interface #OsdkOsal_TaskSleepMs. */
    E_OsdkStat (*TaskSleepMs)(uint32_t timeMs);

    /*! Specifies payload create mutex interface that need register by this format. This interface is mainly used to create
     * mutex. Users need to adapt according to their own platform and system. Also need test the implemented interface
     * before registration to ensure that the registration can be used normally. For details of the parameters, please
     * refer to the following API interface #OsdkOsal_MutexCreate. */
    E_OsdkStat (*MutexCreate)(T_OsdkMutexHandle *mutex);

    /*! Specifies payload destroy the created mutex interface that need register by this format. This interface is mainly
     * used to destroy the created mutex. Users need to adapt according to their own platform and system. Also need test
     * the implemented interface before registration to ensure that the registration can be used normally. For details of
     * the parameters, please refer to the following API interface #OsdkOsal_MutexDestory. */
    E_OsdkStat (*MutexDestroy)(T_OsdkMutexHandle mutex);

    /*! Specifies payload lock the created mutex interface that need register by this format. This interface is mainly used to lock
     * the created mutex. Users need to adapt according to their own platform and system. Also need test the implemented interface
     * before registration to ensure that the registration can be used normally. For details of the parameters, please
     * refer to the following API interface #OsdkOsal_MutexLock. */
    E_OsdkStat (*MutexLock)(T_OsdkMutexHandle mutex);

    /*! Specifies payload unlock the created mutex interface that need register by this format. This interface is mainly
     * used to unlock the created mutex. Users need to adapt according to their own platform and system. Also need test the
     * implemented interface before registration to ensure that the registration can be used normally. For details of the
     * parameters, please refer to the following API interface #OsdkOsal_MutexUnlock. */
    E_OsdkStat (*MutexUnlock)(T_OsdkMutexHandle mutex);

    /*! Specifies payload create semaphore interface that need register by this format. This interface is mainly used to create
     * semaphore. Users need to adapt according to their own platform and system. Also need test the implemented interface
     * before registration to ensure that the registration can be used normally. For details of the parameters, please
     * refer to the following API interface #OsdkOsal_SemaphoreCreate. */
    E_OsdkStat (*SemaphoreCreate)(T_OsdkSemHandle *semaphore, uint32_t initValue);

    /*! Specifies payload destroy the created semaphore interface that need register by this format. This interface is mainly
     * used to destroy the created semaphore. Users need to adapt according to their own platform and system. Also need test
     * the implemented interface before registration to ensure that the registration can be used normally. For details of
     * the parameters, please refer to the following API interface #OsdkOsal_SemaphoreDestroy. */
    E_OsdkStat (*SemaphoreDestroy)(T_OsdkSemHandle semaphore);

    /*! Specifies payload wait the created semaphore forever interface that need register by this format. This interface
     * is mainly used to wait the created semaphore forever. Users need to adapt according to their own platform and
     * system. Also need test the implemented interface before registration to ensure that the registration can be used normally.
     * For details of the parameters, please refer to the following API interface #OsdkOsal_SemaphoreWait. */
    E_OsdkStat (*SemaphoreWait)(T_OsdkSemHandle semaphore);

    /*! Specifies payload wait the created semaphore and set the value of timeout interface that need register by this
     * format. This interface is mainly used to wait the created semaphore. Users need to adapt according to their own platform
     * and system. Also need test the implemented interface before registration to ensure that the registration can be used
     * normally. For details of the parameters, please refer to the following API interface #OsdkOsal_SemaphoreTimedWait. */
    E_OsdkStat (*SemaphoreTimedWait)(T_OsdkSemHandle semaphore, uint32_t waitTimeMs);

    /*! Specifies payload post the created semaphore interface that need register by this format. This interface is mainly
     * used to post the created semaphore. Users need to adapt according to their own platform and system. Also need test
     * the implemented interface before registration to ensure that the registration can be used normally. For details of
     * the parameters, please refer to the following API interface #OsdkOsal_SemaphorePost. */
    E_OsdkStat (*SemaphorePost)(T_OsdkSemHandle semaphore);

    /*! Specifies payload get system millisecond time interface that need register by this format. This interface is mainly
     * used to get system millisecond time. Users need to adapt according to their own platform and system. Also need test
     * the implemented interface before registration to ensure that the registration can be used normally. For details of
     * the parameters, please refer to the following API interface #OsdkOsal_GetTimeMs. */
    E_OsdkStat (*GetTimeMs)(uint32_t *ms);

#ifdef OS_DEBUG
    E_OsdkStat (*GetTimeUs)(uint64_t *us);
#endif

    /*! Specifies payload allocate size bytes of memory interface that need register by this format. This interface is mainly
     * used to allocate size bytes of memory. Users need to adapt according to their own platform and system. Also need test
     * the implemented interface before registration to ensure that the registration can be used normally. For details of
     * the parameters, please refer to the following API interface #OsdkOsal_Malloc. */
    void *(*Malloc)(uint32_t size);

    /*! Specifies payload release allocated memory interface that need register by this format. This interface is mainly
     * used to release allocated memory. Users need to adapt according to their own platform and system. Also need test
     * the implemented interface before registration to ensure that the registration can be used normally. For details of
     * the parameters, please refer to the following API interface #OsdkOsal_Free. */
    void (*Free)(void *ptr);
} T_OsdkOsalHandler;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Register the handler for hal uart interfaces by your platform.
 * @note It should be noted that the interface in hal is written and tested well. Users need to implement all the
 * interfaces. Otherwise, the user interface cannot be successfully registered, and then the user interface is registered
 * through the interface. If the registration fails, it needs to be based on the return code. To judge the problem. Make
 * sure that the feature is available after successful registration. The interface needs to be called at the beginning of
 * the application for registration, otherwise the subsequent functions will not work properly.
 * @param halUartHandler: pointer to the handler for hal uart interfaces by your platform.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkPlatform_RegHalUartHandler(const T_OsdkHalUartHandler *halUartHandler);

#ifdef __linux__
/**
 * @brief Register the handler for hal usb interfaces by your platform.
 * @note It should be noted that the interface in hal is written and tested well. Users need to implement all the
 * interfaces. Otherwise, the user interface cannot be successfully registered, and then the user interface is registered
 * through the interface. If the registration fails, it needs to be based on the return code. To judge the problem. Make
 * sure that the feature is available after successful registration. The interface needs to be called at the beginning of
 * the application for registration, otherwise the subsequent functions will not work properly.
 * @param halUSBBulkHandler: pointer to the handler for hal usb bulk interfaces by your platform.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkPlatform_RegHalUSBBulkHandler(const T_OsdkHalUSBBulkHandler *halUSBBulkHandler);
#endif

/**
 * @brief Register the handler for osal interfaces by your platform.
 * @note It should be noted that the interface in osal is written and tested well. Users need to implement all the
 * interfaces. Otherwise, the user interface cannot be successfully registered, and then the user interface is registered
 * through the interface. If the registration fails, it needs to be based on the return code. To judge the problem. Make
 * sure that the feature is available after successful registration. The interface needs to be called at the beginning of
 * the application for registration, otherwise the subsequent functions will not work properly.
 * @param osalHandler: pointer to the handler for osal interfaces by your platform.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkPlatform_RegOsalHandler(const T_OsdkOsalHandler *osalHandler);

/**
 * @brief Declare the task container, initialize the task, and create task ID.
 * @param pthread_t: pointer to the created task ID.
 * @param taskFunc: pointer to the created task function.
 * @param stackSize: value of task stack size.
 * @param arg: pointer to the user defined data.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_TaskCreate(T_OsdkTaskHandle *task, void *(*taskFunc)(void *),
                                     uint32_t stackSize, void *arg);

/**
 * @brief Destroy the created task.
 * @param pthread_t: pointer to the created task ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_TaskDestroy(T_OsdkTaskHandle task);

/**
 * @brief Let task into a sleep state in a set time, uint:ms.
 * @param timeMs: value of time ms for task sleeping.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_TaskSleepMs(uint32_t timeMs);

/**
 * @brief  Declare the mutex container, initialize the mutex, and create mutex ID.
 * @param  mutex: pointer to the created mutex ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_MutexCreate(T_OsdkMutexHandle *mutex);

/**
 * @brief Destroy the created mutex.
 * @param mutex: pointer to the created mutex ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_MutexDestroy(T_OsdkMutexHandle mutex);

/**
 * @brief Acquire and lock the mutex when peripheral access is required.
 * @param mutex: pointer to the created mutex ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_MutexLock(T_OsdkMutexHandle mutex);

/**
 * @brief Unlock and release the mutex, when done with the peripheral access.
 * @param mutex: pointer to the created mutex ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_MutexUnlock(T_OsdkMutexHandle mutex);

/**
 * @brief  Declare the semaphore container, initialize the semaphore, and create semaphore ID.
 * @param  semaphore: pointer to the created semaphore ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_SemaphoreCreate(T_OsdkSemHandle *semaphore, uint32_t initValue);

/**
 * @brief Destroy the created semaphore.
 * @param semaphore: pointer to the created semaphore ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_SemaphoreDestroy(T_OsdkSemHandle semaphore);

/**
 * @brief Wait the created semaphore forever.
 * @param semaphore: pointer to the created semaphore ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_SemaphoreWait(T_OsdkSemHandle semaphore);

/**
 * @brief Wait the created semaphore and set the value of timeout.
 * @param semaphore: pointer to the created semaphore ID.
 * @param waitTimeMs: value of timeout for waiting created semaphore.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_SemaphoreTimedWait(T_OsdkSemHandle semaphore, uint32_t waitTimeMs);

/**
 * @brief Post the created semaphore.
 * @param semaphore: pointer to the created semaphore ID.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_SemaphorePost(T_OsdkSemHandle semaphore);

/**
 * @brief Get the system millisecond time, uint:ms.
 * @param ms: pointer to the got time ms.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
E_OsdkStat OsdkOsal_GetTimeMs(uint32_t *ms);

/**
 * @brief Get the system time, uint:us.
 * @param us: pointer to the got time us.
 * @return The return code represents the status of the interface execution. For details, please refer to the osdk_typedef.h.
 */
#ifdef OS_DEBUG
E_OsdkStat OsdkOsal_GetTimeUs(uint64_t *us);
#endif

/**
 * @brief Allocate size bytes of memory.
 * @note Users need to apply for memory according to the actual situation of the platform and application. If you do not
 * use the memory after applying for it, please call #OsdkOsal_Free to release the memory.
 * @param size: uint32_t size that need allocate.
 * @return a void pointer to the allocated memory, if equal to NULL, please do related processing to avoid null pointer crash.
 */
void *OsdkOsal_Malloc(uint32_t size);

/**
 * @brief Release allocated memory.
 * @note After calling the #OsdkOsal_Malloc interface, if you do not use memory, please remember to use this interface
 * to release the memory.
 * @param ptr: pointer to the need allocated memory.
 */
void OsdkOsal_Free(void *ptr);

#ifdef __cplusplus
}
#endif

#endif // OSDK_PLATFORM_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
