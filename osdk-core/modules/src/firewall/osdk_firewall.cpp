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

#include "dji_linker.hpp"
#include "osdk_firewall.hpp"
#include "dji_command.hpp"
#include "osdk_device_id.h"
#include "osdk_md5.h"
#include "osdk_policy.hpp"
#include "dji_internal_command.hpp"
#include "dji_log.hpp"

using namespace DJI;
using namespace DJI::OSDK;

Firewall::Firewall(Linker *linker)
    : linker(linker), policyUpdated(false) {
  appKeyBuffer.keyLen = 0;
  OsdkOsal_MutexCreate(&policyUpdatedMutex);
  OsdkOsal_MutexCreate(&appKeyBufferMutex);
  DSTATUS("Firewall is initializing ...");
  static T_RecvCmdItem bulkCmdList[] = {
      PROT_CMD_ITEM(0, 0, V1ProtocolCMD::PSDK::IDVerification[0], V1ProtocolCMD::PSDK::IDVerification[1], MASK_HOST_DEVICE_SET_ID, this, GetIdentityVerifyHandle),
      PROT_CMD_ITEM(0, 0, V1ProtocolCMD::PSDK::uploadPolicyFile[0], V1ProtocolCMD::PSDK::uploadPolicyFile[1], MASK_HOST_DEVICE_SET_ID, this, RequestUploadPolicyFileHandle),
  };
  T_RecvCmdHandle recvCmdHandle;
  recvCmdHandle.cmdList = bulkCmdList;
  recvCmdHandle.cmdCount = sizeof(bulkCmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle.protoType = PROTOCOL_V1;
  if (!linker->registerCmdHandler(&recvCmdHandle)) {
    DERROR("register firewall callback handler failed !");
  }
}

Firewall::~Firewall() {
}

bool Firewall::RequestUpdatePolicy(void) {
  T_CmdInfo reqCmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];
  dji_sdk_upload_policy_file_req uploadPolicyFileReq = {0};

  uploadPolicyFileReq.request_type =
      DJI_UPLOAD_POLICY_FILE_TYPE_REQUEST_FROM_PSDK_LIB;
  reqCmdInfo.sender = linker->getLocalSenderId();
  reqCmdInfo.receiver =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_WIFI, 0x01);
  reqCmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  reqCmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  reqCmdInfo.cmdSet = V1ProtocolCMD::PSDK::uploadPolicyFile[0];
  reqCmdInfo.cmdId = V1ProtocolCMD::PSDK::uploadPolicyFile[1];
  reqCmdInfo.dataLen = sizeof(dji_sdk_upload_policy_file_req);
  reqCmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);

  E_OsdkStat linkAck =
      linker->sendSync(&reqCmdInfo, (uint8_t *) &uploadPolicyFileReq, &ackInfo,
                       ackData, 3000, 3);

  if (linkAck == OSDK_STAT_OK)
    return true;
  else
    return false;
}

uint8_t *OsdkStr_PutStrToBuf(uint8_t *pBuf, char *str, uint32_t maxStrLen) {
  uint32_t len;

  if (strlen(str) > maxStrLen) {
    len = maxStrLen;
  } else {
    len = strlen(str);
  }

  memcpy(pBuf, str, len);

  return (pBuf + len);
}

E_OsdkStat Firewall::GetIdentityVerifyHandle(struct _CommandHandle *cmdHandle,
                                             const T_CmdInfo *cmdInfo,
                                             const uint8_t *cmdData,
                                             void *userData) {
  if (!userData) {
    DERROR("Callback userdata is a null value");
    return OSDK_STAT_ERR_PARAM;
  }
  Firewall *firewall = (Firewall *) userData;
  dji_sdk_id_verification_req
      *req = (dji_sdk_id_verification_req *) cmdData;
  dji_sdk_id_verification_ack ack = {0};
  MD5_CTX md5Ctx;
  uint8_t calBuf[OSDK_IDENTITY_VERIFY_CALMD5_MAX_NUM];
  uint8_t *pCal;

  //key + random = md5
  pCal = calBuf;
  pCal = OsdkStr_PutStrToBuf(pCal, (char *) firewall->appKeyBuffer.data,
                             firewall->appKeyBuffer.keyLen);
  pCal = OsdkStr_PutStrToBuf(pCal, (char *) req->nonce, sizeof(req->nonce));

  OsdkMd5_Init(&md5Ctx);
  OsdkMd5_Update(&md5Ctx, calBuf, (pCal - calBuf));
  OsdkMd5_Final(&md5Ctx, ack.md5);
  ack.ret_code = OSDK_STAT_OK;

  return firewall->linker->sendAck(cmdInfo, (uint8_t *) &ack, sizeof(ack));
}

E_OsdkStat Firewall::RequestUploadPolicyFileHandle(T_CmdHandle *cmdHandle,
                                                   const T_CmdInfo *cmdInfo,
                                                   const uint8_t *cmdData,
                                                   void *userData) {
  if (!userData) {
    DERROR("Callback userdata is a null value");
    return OSDK_STAT_ERR_PARAM;
  }
  Firewall *firewall = (Firewall *) userData;
  dji_sdk_upload_policy_file_req
      *req = (dji_sdk_upload_policy_file_req *) cmdData;
  DSTATUS("request upload policy file type:%d", req->request_type);

  switch (req->request_type) {
    case DJI_UPLOAD_POLICY_FILE_TYPE_REQUEST: {
      MD5_CTX md5Ctx;
      dji_sdk_upload_policy_file_rsp fileInfoAck = {0};
      fileInfoAck.ret_code = OSDK_STAT_OK;
      fileInfoAck.reserved = 0;
      fileInfoAck.version =
          s_osdkPolicyFileBinaryArray[OSDK_POLICY_FILE_VERSION_OFFSET];
      fileInfoAck.total_length = sizeof(s_osdkPolicyFileBinaryArray);

      OsdkMd5_Init(&md5Ctx);
      OsdkMd5_Update(&md5Ctx, s_osdkPolicyFileBinaryArray,
                     sizeof(s_osdkPolicyFileBinaryArray));
      OsdkMd5_Final(&md5Ctx, fileInfoAck.md5);

      DSTATUS("Upload policy file info md5 checksum and version");
      E_OsdkStat ret =
          firewall->linker->sendAck(cmdInfo, (const uint8_t *) &fileInfoAck, sizeof(fileInfoAck));
      if (ret != OSDK_STAT_OK) {
        DERROR("request upload policy file info ack error:%lld", ret);
      }
      break;
    }
    case DJI_UPLOAD_POLICY_FILE_TYPE_TRANSFER: {
      dji_sdk_upload_policy_data_rsp *fileDataAck;
      uint16_t dataLen =
          sizeof(dji_sdk_upload_policy_data_rsp) - 1 + req->data_length;

      fileDataAck = (dji_sdk_upload_policy_data_rsp *) OsdkOsal_Malloc(dataLen);
      if (fileDataAck == NULL) {
        DERROR("memory malloc error");
        return OSDK_STAT_ERR_ALLOC;
      }
      memset(fileDataAck, 0, dataLen);

      DSTATUS("request upload policy file data: %d %d %d", req->data_seq,
             req->data_offset, req->data_length);

      if (req->data_offset < sizeof(s_osdkPolicyFileBinaryArray) &&
          req->data_length < sizeof(s_osdkPolicyFileBinaryArray)) {
        fileDataAck->ret_code = OSDK_STAT_OK;
        fileDataAck->data_seq = req->data_seq;
        memcpy(fileDataAck->data,
               &s_osdkPolicyFileBinaryArray[req->data_offset],
               req->data_length);
      } else {
        fileDataAck->ret_code = OSDK_STAT_ERR_PARAM;
        fileDataAck->data_seq = req->data_seq;
        DERROR("request upload policy file data param error:%lld");
      }

      E_OsdkStat
          ret = firewall->linker->sendAck(cmdInfo, (const uint8_t *) fileDataAck, dataLen);
      if (ret != OSDK_STAT_OK) {
        DERROR("request upload policy file data ack error:%lld", ret);
      }
      OsdkOsal_Free(fileDataAck);
      break;
    }
    case DJI_UPLOAD_POLICY_FILE_TYPE_UPDATED: {
      dji_sdk_upload_policy_data_rsp uploadResultAck = {0};
      if (req->upload_result == DJI_UPLOAD_POLICY_FILE_RESULT_NO_NEED ||
          req->upload_result == DJI_UPLOAD_POLICY_FILE_RESULT_SUCCESS) {
        DSTATUS("request upload policy file success");
        firewall->setPolicyUpdated(true);
      } else if (req->upload_result == DJI_UPLOAD_POLICY_FILE_RESULT_FAILED) {
        DERROR("request upload policy file failed");
      }

      uploadResultAck.ret_code = OSDK_STAT_OK;
      uploadResultAck.data_seq = 0;
      E_OsdkStat ret =
          firewall->linker->sendAck(cmdInfo, (const uint8_t *) &uploadResultAck, sizeof(dji_sdk_upload_policy_data_rsp));
      if (ret != OSDK_STAT_OK) {
        DERROR("request upload policy file result ack error:%lld", ret);
      }
      break;
    }
    default:break;
  }

  return OSDK_STAT_OK;
}

bool Firewall::isPolicyUpdated() {
  OsdkOsal_MutexLock(policyUpdatedMutex);
  bool ret = policyUpdated;
  OsdkOsal_MutexUnlock(policyUpdatedMutex);
  return ret;
}

void Firewall::setPolicyUpdated(bool value) {
  OsdkOsal_MutexLock(policyUpdatedMutex);
  policyUpdated = value;
  OsdkOsal_MutexUnlock(policyUpdatedMutex);
}

osdk_app_key_buffer_type Firewall::getAppKey() {
  OsdkOsal_MutexLock(appKeyBufferMutex);
  osdk_app_key_buffer_type buffer = appKeyBuffer;
  OsdkOsal_MutexUnlock(appKeyBufferMutex);
  return buffer;
}

void Firewall::setAppKey(uint8_t *data, uint8_t len) {
  OsdkOsal_MutexLock(appKeyBufferMutex);
  memcpy(appKeyBuffer.data ,data, len);
  appKeyBuffer.keyLen = len;
  OsdkOsal_MutexUnlock(appKeyBufferMutex);
}
