/*************************************************************************
 > File Name: mop_osal_osdk.c
 > Author: dafeng.xu
 > Created Time: Thu 12 Dec 2019 09:06:13 PM CST
************************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "mop_osal.h"
#include "mop_priv.h"
#include "osdk_typedef.h"
#include "osdk_platform.h"
#include "osdk_msgq.h"
#include "time.h"
#include "signal.h"

#define MOP_OSAL_LOGF(format, ...) \
                        MOP_LOGF("MOP_OSAL:" format, ##__VA_ARGS__)
#define MOP_OSAL_LOGE(format, ...) \
                        MOP_LOGE("MOP_OSAL:" format, ##__VA_ARGS__)
#define MOP_OSAL_LOGW(format, ...) \
                        MOP_LOGW("MOP_OSAL:" format, ##__VA_ARGS__)
#define MOP_OSAL_LOGI(format, ...) \
                        MOP_LOGI("MOP_OSAL:" format, ##__VA_ARGS__)
#define MOP_OSAL_LOGT(format, ...) \
                        MOP_LOGT("MOP_OSAL:" format, ##__VA_ARGS__)
#define MOP_OSAL_LOGV(format, ...) \
                        MOP_LOGV("MOP_OSAL:" format, ##__VA_ARGS__)

int32_t mop_osal_task_create(const char *name, void (*start_routine)(void *),
                            void *arg, uint16_t stack_size, mop_osal_task_t *task)
{
    E_OsdkStat osdkStat;
    osdkStat = OsdkOsal_TaskCreate((T_OsdkTaskHandle *)task, start_routine, stack_size, arg);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk task create failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_task_join(mop_osal_task_t task)
{
    return MOP_ERR_FAILED;
}

int32_t mop_osal_task_destroy(mop_osal_task_t task)
{
    E_OsdkStat osdkStat;
    osdkStat = OsdkOsal_TaskDestroy((T_OsdkTaskHandle)task);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk task destroy failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_msgq_create(const char *name, uint32_t length, mop_osal_msgq_t *msgq)
{
    T_msgqAttrib attrib;
    attrib.name = name;
    attrib.bufSize = length;
    E_OsdkStat osdkStat = OsdkMsgq_Create(&attrib, (T_msgQueue **)msgq);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk msgq create failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_msgq_destroy(mop_osal_msgq_t msgq)
{
    E_OsdkStat osdkStat = OsdkMsgq_Destroy((T_msgQueue *)msgq);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk msgq destroy failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_msgq_send(mop_osal_msgq_t msgq, void *message, uint32_t size, uint32_t timeout_ms)
{
    E_OsdkStat osdkStat = OsdkMsgq_Send((T_msgQueue *)msgq, message, size, timeout_ms);

    if(osdkStat == OSDK_STAT_OK) {
        return MOP_SUCCESS;
    }else if(osdkStat == OSDK_STAT_ERR_TIMEOUT) {
        return MOP_ERR_TIMEOUT;
    }else {
        MOP_OSAL_LOGE("osdk msgq send failed");
        return MOP_ERR_FAILED;
    }
}

int32_t mop_osal_msgq_recv(mop_osal_msgq_t msgq, void *buffer, uint32_t *size, uint32_t timeout_ms)
{
    E_OsdkStat osdkStat = OsdkMsgq_Recv((T_msgQueue *)msgq, (uint8_t *)buffer, size, timeout_ms);

    if(osdkStat == OSDK_STAT_OK) {
        return MOP_SUCCESS;
    }else if(osdkStat == OSDK_STAT_ERR_TIMEOUT) {
        return MOP_ERR_TIMEOUT;
    }else if(osdkStat == OSDK_STAT_NOT_READY) {
        return MOP_ERR_NOTREADY;
    }else {
        MOP_OSAL_LOGE("osdk msgq recv failed");
        return MOP_ERR_FAILED;
    }
}

int32_t mop_osal_msgq_get_count(mop_osal_msgq_t msgq, uint32_t *count)
{
    E_OsdkStat osdkStat = OsdkMsgq_GetCount((T_msgQueue *)msgq, count);
    if (osdkStat != OSDK_STAT_OK) {
        MOP_OSAL_LOGE("osdk msgq get count failed");
        return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_msleep(uint32_t time_ms)
{
    E_OsdkStat osdkStat = OsdkOsal_TaskSleepMs(time_ms);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk msleep failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_get_time_ms(uint32_t *time_ms)
{
    E_OsdkStat osdkStat = OsdkOsal_GetTimeMs(time_ms);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk get time ms failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_sema_create(mop_osal_sema_t *sema, uint32_t init_cnt)
{
    E_OsdkStat osdkStat = OsdkOsal_SemaphoreCreate((T_OsdkSemHandle *)sema, init_cnt);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk sema create failed");
       return MOP_ERR_TIMEOUT;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_sema_destroy(mop_osal_sema_t sema)
{
    E_OsdkStat osdkStat = OsdkOsal_SemaphoreDestroy((T_OsdkSemHandle)sema);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk sema destroy failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_sema_wait(mop_osal_sema_t sema, uint32_t timeout_ms)
{
    E_OsdkStat osdkStat;

    if(timeout_ms == MOP_WAIT_FOREVER) {
        osdkStat = OsdkOsal_SemaphoreWait((T_OsdkSemHandle)sema);
    } else {
        osdkStat = OsdkOsal_SemaphoreTimedWait((T_OsdkSemHandle)sema, timeout_ms);
    }

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk sema timed wait failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_sema_post(mop_osal_sema_t sema)
{
    E_OsdkStat osdkStat = OsdkOsal_SemaphorePost((T_OsdkSemHandle)sema);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk sema post failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_mutex_create(mop_osal_mutex_t *mutex)
{
    E_OsdkStat osdkStat = OsdkOsal_MutexCreate((T_OsdkMutexHandle *)mutex);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk mutex create failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_mutex_lock(mop_osal_mutex_t mutex)
{
    E_OsdkStat osdkStat = OsdkOsal_MutexLock((T_OsdkMutexHandle)mutex);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk mutex lock failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_mutex_unlock(mop_osal_mutex_t mutex)
{
    E_OsdkStat osdkStat = OsdkOsal_MutexUnlock((T_OsdkMutexHandle)mutex);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk mutex unlock failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t mop_osal_mutex_destroy(mop_osal_mutex_t mutex)
{
    E_OsdkStat osdkStat = OsdkOsal_MutexDestroy((T_OsdkMutexHandle)mutex);

    if(osdkStat != OSDK_STAT_OK) {
       MOP_OSAL_LOGE("osdk mutex destroy failed");
       return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

void *mop_osal_malloc(uint32_t size)
{
    return OsdkOsal_Malloc(size);
}

void mop_osal_free(void* ptr)
{
    return OsdkOsal_Free(ptr);
}

int32_t mop_osal_timer_create(void (*timer_entry)(void), uint32_t timeout_ms, mop_osal_timer_t *timer)
{
    return MOP_SUCCESS;
}

int32_t mop_osal_timer_destroy(mop_osal_timer_t timer)
{
    return MOP_SUCCESS;
}