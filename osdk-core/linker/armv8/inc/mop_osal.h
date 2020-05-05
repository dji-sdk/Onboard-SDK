/*************************************************************************
 > File Name: mop.h
 > Author: dafeng.xu
 > Created Time: Mon 09 Dec 2019 12:09:58 PM CST
************************************************************************/

#ifndef  __MOP_OSAL_H__
#define __MOP_OSAL_H__

#include "mop_platform.h"
#include "stdint.h"
#include "stdbool.h"
#include "assert.h"

typedef void *mop_osal_task_t;
typedef void *mop_osal_msgq_t;
typedef void *mop_osal_sema_t;
typedef void *mop_osal_mutex_t;
typedef void *mop_osal_timer_t;

#define UNUSED(x) (x = x)

#ifdef __cplusplus
extern "C" {
#endif

int32_t mop_osal_task_create(const char *name, void (*start_routine)(void *),
                             void *arg, uint16_t stack_size,
                             mop_osal_task_t *task);
int32_t mop_osal_task_join(mop_osal_task_t task);
int32_t mop_osal_task_destroy(mop_osal_task_t task);

int32_t mop_osal_msleep(uint32_t time_ms);
int32_t mop_osal_get_time_ms(uint32_t *time_ms);

int32_t mop_osal_msgq_create(const char *name, uint32_t length, mop_osal_msgq_t *msgq);
int32_t mop_osal_msgq_destroy(mop_osal_msgq_t msgq);
int32_t mop_osal_msgq_send(mop_osal_msgq_t msgq, void *message,
                          uint32_t size, uint32_t timeout_ms);
int32_t mop_osal_msgq_recv(mop_osal_msgq_t msgq, void *buffer,
                          uint32_t *size, uint32_t timeout_ms);
int32_t mop_osal_msgq_get_count(mop_osal_msgq_t msgq, uint32_t *count);

int32_t mop_osal_sema_create(mop_osal_sema_t *sema, uint32_t init_cnt);
int32_t mop_osal_sema_destroy(mop_osal_sema_t sema);
int32_t mop_osal_sema_wait(mop_osal_sema_t seam, uint32_t timeout_ms);
int32_t mop_osal_sema_post(mop_osal_sema_t seam);

int32_t mop_osal_mutex_create(mop_osal_mutex_t *mutex);
int32_t mop_osal_mutex_lock(mop_osal_mutex_t mutex);
int32_t mop_osal_mutex_unlock(mop_osal_mutex_t mutex);
int32_t mop_osal_mutex_destroy(mop_osal_mutex_t mutex);

int32_t mop_osal_timer_create(void (*timer_entry)(void), uint32_t timeout_ms,
                             mop_osal_timer_t *timer);
int32_t mop_osal_timer_destroy(mop_osal_timer_t timer);

void *mop_osal_malloc(uint32_t size);
void mop_osal_free(void *ptr);

#ifdef __cplusplus
}
#endif
#endif