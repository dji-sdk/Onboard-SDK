/** @file osdk_firewall.cpp
 *  @version 4.0
 *  @date March 2020
 *
 *  @brief Implementation for osdk firewall
 *
 *  @Copyright (c) 2020 DJI
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

#ifndef OSDK_FIREWALL_HPP
#define OSDK_FIREWALL_HPP

#include "osdk_command.h"

typedef enum {
  DJI_UPLOAD_POLICY_FILE_TYPE_REQUEST = 0, // 策略文件更新请求
  DJI_UPLOAD_POLICY_FILE_TYPE_TRANSFER = 1, // 策略文件更新过程中
  DJI_UPLOAD_POLICY_FILE_TYPE_UPDATED = 2, // 策略文件更新完成
  DJI_UPLOAD_POLICY_FILE_TYPE_REQUEST_FROM_PSDK_LIB = 3,
} DJI_UPLOAD_POLICY_FILE_TYPE;

typedef enum {
  DJI_UPLOAD_POLICY_FILE_RESULT_NO_NEED = 0, // 无需更新
  DJI_UPLOAD_POLICY_FILE_RESULT_SUCCESS = 1, // 更新成功
  DJI_UPLOAD_POLICY_FILE_RESULT_FAILED = 255, // 更新失败
} DJI_UPLOAD_POLICY_FILE_RESULT;

#pragma pack(1)

typedef struct {
  // 请求数据类型：0：请求更新 1：传输数据 2 :  传输完成 DJI_UPLOAD_POLICY_FILE_TYPE
  uint8_t request_type;
  // 保留位
  uint8_t upload_result;
  // 请求传输的数据包的seq（type=1时有效）
  uint16_t data_seq;
  // 请求传输数据的偏移位置（type=1时有效）
  uint32_t data_offset;
  // 请求传输数据的长度（type=1时有效）
  uint32_t data_length;
} dji_sdk_upload_policy_file_req;

typedef struct {
  // 返回码
  uint8_t ret_code;
  // 策略文件版本（type=0时有效）
  uint8_t version;
  // 保留位（type=0时有效）
  uint8_t reserved;
  // 策略文件长度（type=0时有效）
  uint32_t total_length;
  // 策略文件MD5值（type=0时有效）
  uint8_t md5[16];
} dji_sdk_upload_policy_file_rsp;

typedef struct {
  uint8_t ret_code;
  // 当前数据包的序号（type=1时有效）
  uint16_t data_seq;
  // 数据段（type=1时有效）
  uint8_t data[1];
} dji_sdk_upload_policy_data_rsp;


typedef struct {
  // 随机数
  uint8_t nonce[16];
} dji_sdk_id_verification_req;

typedef struct {
  // 返回码
  uint8_t ret_code;
  // md5值
  uint8_t md5[16];
} dji_sdk_id_verification_ack;

typedef struct {
  uint8_t data[0xFF];
  uint8_t keyLen;
} osdk_app_key_buffer_type;

#pragma pack()

namespace DJI {
namespace OSDK {

// Forward Declaration
class Linker;

class Firewall {
 public:
  Firewall(Linker *linker);
  ~Firewall();
  bool RequestUpdatePolicy(void);
  static E_OsdkStat GetIdentityVerifyHandle(struct _CommandHandle *cmdHandle,
                                            const T_CmdInfo *cmdInfo,
                                            const uint8_t *cmdData,
                                            void *userData);
  static E_OsdkStat RequestUploadPolicyFileHandle(T_CmdHandle *cmdHandle,
                                                  const T_CmdInfo *cmdInfo,
                                                  const uint8_t *cmdData,
                                                  void *userData);
  bool isPolicyUpdated();
  void setAppKey(uint8_t *data, uint8_t len);
 private:
  Linker *linker;
  bool policyUpdated;
  osdk_app_key_buffer_type appKeyBuffer;

  void setPolicyUpdated(bool value);
  osdk_app_key_buffer_type getAppKey();

 private:
  T_OsdkMutexHandle policyUpdatedMutex;
  T_OsdkMutexHandle appKeyBufferMutex;
};
}
}

#endif  // OSDK_FIREWALL_HPP
