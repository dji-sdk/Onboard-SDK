/** @file mop_entry_osdk.h
 *  @version 4.0
 *  @date April 2020
 *
 *  @brief
 *  Initialization and deinitialization implementation of MOP.
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
