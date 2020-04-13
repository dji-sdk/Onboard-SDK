/*************************************************************************
 > File Name: mop_entry_osdk.h
 > Author:pakchoi.zhang@dji.com
 > Created Time: Mon 09 Dec 2019 12:09:58 PM CST
************************************************************************/

#ifndef  __MOP_ENTRY_OSDK_H__
#define __MOP_ENTRY_OSDK_H__

#include <stdint.h>

#define MOP_ENTRY_LOGF(format, ...) \
                        MOP_LOGF("MOP_ENTRY:" format, ##__VA_ARGS__)
#define MOP_ENTRY_LOGE(format, ...) \
                        MOP_LOGE("MOP_ENTRY:" format, ##__VA_ARGS__)
#define MOP_ENTRY_LOGW(format, ...) \
                        MOP_LOGW("MOP_ENTRY:" format, ##__VA_ARGS__)
#define MOP_ENTRY_LOGI(format, ...) \
                        MOP_LOGI("MOP_ENTRY:" format, ##__VA_ARGS__)
#define MOP_ENTRY_LOGT(format, ...) \
                        MOP_LOGT("MOP_ENTRY:" format, ##__VA_ARGS__)
#define MOP_ENTRY_LOGV(format, ...) \
                        MOP_LOGV("MOP_ENTRY:" format, ##__VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

int32_t mop_osdk_init(void);
int32_t mop_osdk_deinit(void);

#ifdef __cplusplus
}
#endif
#endif
