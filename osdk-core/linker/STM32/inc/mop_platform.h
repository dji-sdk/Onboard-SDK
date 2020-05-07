#ifndef __MOP_PLATFORM_H_
#define __MOP_PLATFORM_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#if PSDK_ARCH_SYS_LINUX

#include "logger/psdk_logger_internal.h"

#define MOP_LOGF(...)    PSDK_LOG_ERROR(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGE(...)    PSDK_LOG_ERROR(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGW(...)    PSDK_LOG_WARN(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGI(...)    PSDK_LOG_INFO(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGT(...)    PSDK_LOG_DEBUG(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGV(...)    PSDK_LOG_DEBUG(MODULE_NAME_MOP, __VA_ARGS__)
#endif

#ifdef PLATFORM_OSDK
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include "osdk_command.h"
#include "osdk_command_instance.h"
#include "osdk_logger_internal.h"

#define MOP_LOGF(...) OSDK_LOG_ERROR(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGE(...) OSDK_LOG_ERROR(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGW(...) OSDK_LOG_WARN(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGI(...) OSDK_LOG_INFO(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGT(...) OSDK_LOG_DEBUG(MODULE_NAME_MOP, __VA_ARGS__)
#define MOP_LOGV(...) OSDK_LOG_DEBUG(MODULE_NAME_MOP, __VA_ARGS__)

#endif

#ifdef PLATFORM_DUSS
#include <string.h>
#include "duml_log.h"
#include "duml_sketch.h"
#include "duml_hal_core.h"
#include "duml_mb_pack.h"
#include "usb_bulk_raw_multich_v2.h"

#define MOP_LOGF(...) DUSS_LOGF(DUSS_MODULE_SYSTEM, __VA_ARGS__)
#define MOP_LOGE(...) DUSS_LOGE(DUSS_MODULE_SYSTEM, __VA_ARGS__)
#define MOP_LOGW(...) DUSS_LOGW(DUSS_MODULE_SYSTEM, __VA_ARGS__)
#define MOP_LOGI(...) DUSS_LOGI(DUSS_MODULE_SYSTEM, __VA_ARGS__)
#define MOP_LOGT(...) DUSS_LOGT(DUSS_MODULE_SYSTEM, __VA_ARGS__)
#define MOP_LOGV(...) DUSS_LOGV(DUSS_MODULE_SYSTEM, __VA_ARGS__)

#endif

#ifdef __APPLE__

#include "stdbool.h"
#include <assert.h>
#include <string.h>
#include <mach/mach_init.h>
#include <mach/semaphore.h>
#include <mach/task.h>

typedef enum {
    // Only debug mode can print.
    MOPLogTagTypeF,
    // Only debug and release mode can print.
    MOPLogTagTypeError,
    // Only debug mode can print.
    MOPLogTagTypeWarning,
    // Only debug mode can print.
    MOPLogTagTypeInfo,
    // Only debug mode can print.
    MOPLogTagTypeT,
    // Only debug mode can print.
    MOPLogTagTypeVerbose
} MOPLogTagType;

typedef void(*DJIMOPLogCenterPrintPointer) (const char* file_path, int logline, MOPLogTagType tag, const char *fmt, ...);

extern DJIMOPLogCenterPrintPointer mop_log_center_print;

#ifdef __cplusplus
extern "C" {
#endif

void set_mop_log_center_print_pointer(DJIMOPLogCenterPrintPointer pointer);

#ifdef __cplusplus
}
#endif

#define MOP_LOGF(...) mop_log_center_print(__FILE__, __LINE__, MOPLogTagTypeF, __VA_ARGS__)
#define MOP_LOGE(...) mop_log_center_print(__FILE__, __LINE__, MOPLogTagTypeError, __VA_ARGS__)
#define MOP_LOGW(...) mop_log_center_print(__FILE__, __LINE__, MOPLogTagTypeWarning, __VA_ARGS__)
#define MOP_LOGI(...) mop_log_center_print(__FILE__, __LINE__, MOPLogTagTypeInfo, __VA_ARGS__)
#define MOP_LOGT(...) mop_log_center_print(__FILE__, __LINE__, MOPLogTagTypeT, __VA_ARGS__)
#define MOP_LOGV(...) mop_log_center_print(__FILE__, __LINE__, MOPLogTagTypeVerbose, __VA_ARGS__)

#else

#include <semaphore.h>

#endif


#ifdef __ANDROID__
#include <stdbool.h>
#include <assert.h>
#include <android/log.h>
#define LOG_TAG "DJI_MOP_JNI"

typedef void(*MopLog) (int prio, const char* tag, const char* fmt, ...);

extern MopLog mopLog;

#ifdef __cplusplus
extern "C" {
#endif

void set_mop_log_handle(const MopLog mopLog);

#ifdef __cplusplus
}
#endif

#define UNUSED(x) (x = x)
#define MSDK_USE_LOG            true

#if MSDK_USE_LOG

#define MOP_LOGF(fmt, ...) mopLog( ANDROID_LOG_FATAL, LOG_TAG, "[Fatal]-[%s:%d]" fmt, __FILE__, __LINE__ , ##__VA_ARGS__)
#define MOP_LOGE(fmt, ...) mopLog( ANDROID_LOG_ERROR, LOG_TAG, "[Error]-[%s:%d]" fmt, __FILE__, __LINE__ ,##__VA_ARGS__)
#define MOP_LOGW(fmt, ...) mopLog( ANDROID_LOG_WARN, LOG_TAG, "[Warn]-[%s:%d]" fmt, __FILE__, __LINE__ ,##__VA_ARGS__)
#define MOP_LOGI(fmt, ...) mopLog( ANDROID_LOG_INFO, LOG_TAG, "[Info]-[%s:%d]" fmt, __FILE__, __LINE__ ,##__VA_ARGS__)
#define MOP_LOGT(fmt, ...) mopLog( ANDROID_LOG_INFO, LOG_TAG, "[T]-[%s:%d]" fmt, __FILE__, __LINE__ ,##__VA_ARGS__)
#define MOP_LOGV(fmt, ...) mopLog( ANDROID_LOG_VERBOSE, LOG_TAG, "[Verbose]-[%s:%d]" fmt, __FILE__, __LINE__ ,##__VA_ARGS__)

#else
#define MOP_LOGF(...) mopLog( ANDROID_LOG_FATAL, LOG_TAG, ##__VA_ARGS__)
#define MOP_LOGE(...) mopLog( ANDROID_LOG_ERROR, LOG_TAG, ##__VA_ARGS__)
#define MOP_LOGW(...) mopLog( ANDROID_LOG_WARN, LOG_TAG, ##__VA_ARGS__)
#define MOP_LOGI(...) mopLog( ANDROID_LOG_INFO, LOG_TAG, ##__VA_ARGS__)
#define MOP_LOGT(...) mopLog( ANDROID_LOG_INFO, LOG_TAG, ##__VA_ARGS__)
#define MOP_LOGV(...) mopLog( ANDROID_LOG_VERBOSE, LOG_TAG, ##__VA_ARGS__)
#endif

#endif
#endif
