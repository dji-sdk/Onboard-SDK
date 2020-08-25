/*! @file om_download_sample.cpp
 *  @version 4.0.0
 *  @date June 16 2020
 *
 *  @brief Sample to show how MSDK downloads files from OSDK
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

#include "dji_linux_helpers.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <dirent.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "mop_sample_simple_protocol.hpp"
#include <openssl/md5.h>

using namespace DJI::OSDK;

#define ASSERT_MOP_RET(ret) \
  if (ret != MOP_PASSED && ret != MOP_TIMEOUT) { \
  ErrorCode::printErrorCodeMsg(ret); \
  printf("ret = %d\n", ret);\
  throw std::runtime_error("MOP result failed"); \
}

/* Private constants ---------------------------------------------------------*/
#define TEST_OM_UNRELIABLE_PIPELINE_ID                    49154
#define TEST_OM_UNRELIABLE_TRANSFOR_FREQ                  40
#define UNRELIABLE_READ_ONCE_BUFFER_SIZE                  (100 * 1024)
#define UNRELIABLE_SEND_ONCE_BUFFER_SIZE                  (100 * 1024)

#define TEST_OM_RELIABLE_PIPELINE_ID                      49155
#define RELIABLE_SEND_ONCE_BUFFER_SIZE                    (2 * 1024 * 1024)
#define RELIABLE_RECV_ONCE_BUFFER_SIZE                    (100 * 1024)
#define TEST_MOP_CHANNEL_FILE_SERVICE_FILE_PATH           "test.mp4"

/* Private types -------------------------------------------------------------*/
typedef enum {
    MOP_FILE_SERVICE_DOWNLOAD_IDEL = 0,
    MOP_FILE_SERVICE_DOWNLOAD_REQUEST_START,
    MOP_FILE_SERVICE_DOWNLOAD_FILE_INFO_SUCCESS,
    MOP_FILE_SERVICE_DOWNLOAD_FILE_INFO_FAILED,
    MOP_FILE_SERVICE_DOWNLOAD_DATA_SENDING,
    MOP_FILE_SERVICE_DOWNLOAD_FINISHED_SUCCESS,
    MOP_FILE_SERVICE_DOWNLOAD_FINISHED_FAILED,
} E_MopFileServiceDownloadState;

typedef enum {
    MOP_FILE_SERVICE_UPLOAD_IDEL = 0,
    MOP_FILE_SERVICE_UPLOAD_REQUEST_START,
    MOP_FILE_SERVICE_UPLOAD_FILE_INFO_SUCCESS,
    MOP_FILE_SERVICE_UPLOAD_FILE_INFO_FAILED,
    MOP_FILE_SERVICE_UPLOAD_DATA_SENDING,
    MOP_FILE_SERVICE_UPLOAD_FINISHED_SUCCESS,
    MOP_FILE_SERVICE_UPLOAD_FINISHED_FAILED,
} E_MopFileServiceUploadState;

typedef struct {
    E_MopFileServiceDownloadState downloadState;
    uint16_t downloadSeqNum;
    E_MopFileServiceUploadState uploadState;
    uint16_t uploadSeqNum;
} T_MopFileServiceContent;

/* Private values -------------------------------------------------------------*/
static T_MopFileServiceContent s_fileServiceContent;

static bool isReliable = false;
static bool sampleFinish = false;
static bool isAccept = false;
static uint32_t total_len = 0;

static void* MopChannelFileServiceSendTask(void *arg)
{
    MopPipeline *OM_Pipeline = (MopPipeline *)arg;
    if (!OM_Pipeline)
        std::runtime_error("Error param");
    int returnCode;
    MopErrCode mopRet;
    uint32_t sendRealLen;
    uint8_t *sendBuf;
    MD5_CTX downloadFileMd5Ctx;
    FILE *downloadFile = NULL;
    uint8_t downloadFileMd5[16];
    uint32_t downloadFileTotalSize = 0;
    uint32_t downloadWriteLen = 0;
    uint16_t downloadPackCount = 0;
    fileInfo downloadFileInfo = {0};
    sampleProtocolStruct ack = {0};
    uint32_t downloadStartMs = 0;
    uint32_t downloadEndMs = 0;
    float downloadRate = 0;

    MopPipeline::DataPackType fileInfoPack = {.data = (uint8_t *) &ack, .length = UTIL_OFFSETOF(sampleProtocolStruct, data)};

    sendBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_SEND_ONCE_BUFFER_SIZE);
    if (sendBuf == NULL) {
        DERROR("[File-Service] OsdkOsal_Malloc send buffer error");
        sampleFinish = true;
        return NULL;
    }

    while (sampleFinish == false) {
        switch (s_fileServiceContent.uploadState) {
            case MOP_FILE_SERVICE_UPLOAD_REQUEST_START:
                ack.cmd = FILE_TRANSFOR_CMD_ACK;
                ack.subcmd = FILE_TRANSFOR_SUBCMD_ACK_OK;
                ack.seq = s_fileServiceContent.uploadSeqNum;
                ack.dataLen = 0;
                fileInfoPack.data = (uint8_t *) &ack;
                fileInfoPack.length = UTIL_OFFSETOF(sampleProtocolStruct, data);
                mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
                ASSERT_MOP_RET(mopRet)
                DSTATUS("[File-Service] upload request ack");
                s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_IDEL;

                break;
            case MOP_FILE_SERVICE_UPLOAD_FILE_INFO_SUCCESS:
                ack.cmd = FILE_TRANSFOR_CMD_ACK;
                ack.subcmd = FILE_TRANSFOR_SUBCMD_ACK_OK;
                ack.seq = s_fileServiceContent.uploadSeqNum;
                ack.dataLen = 0;
                fileInfoPack.data = (uint8_t *) &ack;
                fileInfoPack.length = UTIL_OFFSETOF(sampleProtocolStruct, data);
                mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
                ASSERT_MOP_RET(mopRet)
                DSTATUS("[File-Service] upload file info success");
                s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_IDEL;

                break;
            case MOP_FILE_SERVICE_UPLOAD_FILE_INFO_FAILED:
                ack.cmd = FILE_TRANSFOR_CMD_ACK;
                ack.subcmd = FILE_TRANSFOR_SUBCMD_ACK_REJECTED;
                ack.seq = s_fileServiceContent.uploadSeqNum;
                ack.dataLen = 0;
                fileInfoPack.data = (uint8_t *) &ack;
                fileInfoPack.length = UTIL_OFFSETOF(sampleProtocolStruct, data);
                mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
                ASSERT_MOP_RET(mopRet)
                DSTATUS("[File-Service]  upload file info failed");
                s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_IDEL;

                break;
            case MOP_FILE_SERVICE_UPLOAD_FINISHED_SUCCESS:
                ack.cmd = FILE_TRANSFOR_CMD_RESULT;
                ack.subcmd = FILE_TRANSFOR_SUBCMD_RESULT_OK;
                ack.seq = s_fileServiceContent.uploadSeqNum++;
                ack.dataLen = 0;
                fileInfoPack.data = (uint8_t *) &ack;
                fileInfoPack.length = UTIL_OFFSETOF(sampleProtocolStruct, data);
                mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
                ASSERT_MOP_RET(mopRet)
                DSTATUS("[File-Service]  upload finished success");
                s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_IDEL;

                break;
            case MOP_FILE_SERVICE_UPLOAD_FINISHED_FAILED:
                ack.cmd = FILE_TRANSFOR_CMD_RESULT;
                ack.subcmd = FILE_TRANSFOR_SUBCMD_RESULT_FAILED;
                ack.seq = s_fileServiceContent.uploadSeqNum++;
                ack.dataLen = 0;
                fileInfoPack.data = (uint8_t *) &ack;
                fileInfoPack.length = UTIL_OFFSETOF(sampleProtocolStruct, data);
                mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
                ASSERT_MOP_RET(mopRet)
                DSTATUS("[File-Service]  upload finished failed");
                s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_IDEL;
                break;
            default:
                break;
        }

        switch (s_fileServiceContent.downloadState) {
            case MOP_FILE_SERVICE_DOWNLOAD_REQUEST_START:
                ack.cmd = FILE_TRANSFOR_CMD_ACK;
                ack.subcmd = FILE_TRANSFOR_SUBCMD_ACK_OK;
                ack.seq = s_fileServiceContent.downloadSeqNum;
                ack.dataLen = 0;
                fileInfoPack.data = (uint8_t *) &ack;
                fileInfoPack.length = UTIL_OFFSETOF(sampleProtocolStruct, data);
                mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
                ASSERT_MOP_RET(mopRet)
                DSTATUS("[File-Service]  download request ack");
                s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_IDEL;

                break;
            case MOP_FILE_SERVICE_DOWNLOAD_FILE_INFO_SUCCESS:
            {
                MD5_Init(&downloadFileMd5Ctx);
                OsdkOsal_GetTimeMs(&downloadStartMs);
                downloadFile = fopen(TEST_MOP_CHANNEL_FILE_SERVICE_FILE_PATH, "rb");
                if (downloadFile == NULL) {
                    DERROR("[File-Service]  download open file error");
                    sampleFinish = true;
                    return NULL;
                }

                downloadFileTotalSize = 0;

                while (1) {
                    returnCode = fseek(downloadFile, downloadFileTotalSize, SEEK_SET);
                    if (returnCode != 0) {
                        DERROR(
                            "[File-Service]  mop channel fseek file data fail.");
                    }
                    downloadWriteLen = fread(sendBuf, 1, RELIABLE_SEND_ONCE_BUFFER_SIZE,
                                             downloadFile);
                    if (downloadWriteLen > 0) {
                        downloadFileTotalSize += downloadWriteLen;
                        MD5_Update(&downloadFileMd5Ctx, sendBuf, downloadWriteLen);
                        if (downloadWriteLen < RELIABLE_SEND_ONCE_BUFFER_SIZE) {
                            break;
                        }
                    }
                }
                MD5_Final(downloadFileMd5, &downloadFileMd5Ctx);

                sampleProtocolStruct fileInfo = {0};
                fileInfo.cmd = FILE_TRANSFOR_CMD_FILE_INFO;
                fileInfo.subcmd = FILE_TRANSFOR_SUBCMD_DOWNLOAD_REQUEST;
                fileInfo.seq = s_fileServiceContent.downloadSeqNum;
                fileInfo.dataLen = sizeof(fileInfo) - UTIL_OFFSETOF(sampleProtocolStruct, data);

                fileInfo.data.info.isExist = true;
                downloadFileInfo.fileLength = downloadFileTotalSize;
                fileInfo.data.info.fileLength = downloadFileTotalSize;
                strcpy(fileInfo.data.info.fileName, "test.mp4");

                memcpy(&fileInfo.data.info.md5Buf, &downloadFileMd5, sizeof(downloadFileMd5));
                fileInfoPack.data = (uint8_t *) &fileInfo;
                fileInfoPack.length = sizeof(sampleProtocolStruct);
                mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
                ASSERT_MOP_RET(mopRet)
                DSTATUS(
                    "[File-Service]  download ack file info exist:%d length:%d name:%s md5:%02X %d",
                    
                    fileInfo.data.info.isExist,
                    fileInfo.data.info.fileLength,
                    fileInfo.data.info.fileName,
                    fileInfo.data.info.md5Buf[15],
                    sizeof(sampleProtocolStruct));

                s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_DATA_SENDING;

                downloadFileTotalSize = 0;
                downloadPackCount = 0;
                break;
            }
            case MOP_FILE_SERVICE_DOWNLOAD_FILE_INFO_FAILED:
                break;
            case MOP_FILE_SERVICE_DOWNLOAD_DATA_SENDING:
            {
                returnCode = fseek(downloadFile, downloadFileTotalSize, SEEK_SET);
                if (returnCode != 0) {
                    DERROR("[File-Service]  download fseek file data fail.");
                    break;
                }
                downloadWriteLen = fread(&sendBuf[UTIL_OFFSETOF(sampleProtocolStruct, data)],
                                         1,
                                         (RELIABLE_SEND_ONCE_BUFFER_SIZE -
                                          UTIL_OFFSETOF(sampleProtocolStruct, data)),
                                         downloadFile);

                if (downloadWriteLen > 0) {
                    downloadFileTotalSize += downloadWriteLen;
                    downloadPackCount++;
                    DSTATUS(
                        "[File-Service]  download read data from file success, len = %d count:%d",
                        
                        downloadWriteLen, downloadPackCount);
                    sampleProtocolStruct fileData = {0};

                    fileData.cmd = FILE_TRANSFOR_CMD_FILE_DATA;
                    fileData.dataLen = downloadWriteLen;
                    fileData.seq++;
                    if (downloadWriteLen ==
                        (RELIABLE_SEND_ONCE_BUFFER_SIZE -
                         UTIL_OFFSETOF(sampleProtocolStruct, data))) {
                        fileData.subcmd = FILE_TRANSFOR_SUBCMD_FILE_DATA_NORMAL;
                    } else {
                        fileData.subcmd = FILE_TRANSFOR_SUBCMD_FILE_DATA_END;
                    }

                    memcpy(sendBuf, &fileData, UTIL_OFFSETOF(sampleProtocolStruct, data));
RESEND:
                    DSTATUS("[File-Service]  download send data:%d %d %d %d", fileData.cmd,
                                            fileData.subcmd, fileData.seq, fileData.dataLen);
                    fileInfoPack.data = (uint8_t *) sendBuf;
                    fileInfoPack.length = downloadWriteLen + UTIL_OFFSETOF(sampleProtocolStruct, data);
                    mopRet = OM_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);

                    if (mopRet != MOP_PASSED) {
                        DERROR(
                            "[File-Service]  download send file data error,stat:%d",
                             mopRet);
                        if (mopRet == MOP_CONNECTIONCLOSE) {
                            break;
                        } else {
                            goto RESEND;
                        }
                    } else {
                        DSTATUS(
                            "[File-Service]  download send file data length:%d count:%d percent: %.1f %lld",
                            fileInfoPack.length, downloadPackCount,
                            (float) (downloadFileTotalSize * 100) /
                            (float) downloadFileInfo.fileLength);
                    }

                    if (fileData.subcmd == FILE_TRANSFOR_SUBCMD_FILE_DATA_END) {
                        OsdkOsal_GetTimeMs(&downloadEndMs);
                        downloadRate =
                            (float) downloadFileInfo.fileLength * 1000 / (float) (downloadEndMs - downloadStartMs);
                        DSTATUS(
                            "[File-Service]  download finished totalTime:%d, rate:%.2f Byte/s",
                             (downloadEndMs - downloadStartMs), downloadRate);
                    }
                }
         }
                break;
        case MOP_FILE_SERVICE_DOWNLOAD_FINISHED_SUCCESS:
            break;
        case MOP_FILE_SERVICE_DOWNLOAD_FINISHED_FAILED:
            break;
        default:
            break;
        }
    }
    sampleFinish = true;

    return NULL;
}

static void* MopChannelFileServiceRecvTask(void *arg)
{
    MopPipeline *OM_Pipeline = (MopPipeline *)arg;
    if (!OM_Pipeline)
        std::runtime_error("Error param");
    MopErrCode mopRet;
    int returnCode;
    uint32_t recvRealLen;
    uint8_t *recvBuf;
    MD5_CTX uploadFileMd5Ctx;
    FILE *uploadFile = NULL;
    uint8_t uploadFileMd5[16];
    uint32_t uploadFileTotalSize = 0;
    uint32_t uploadWriteLen = 0;
    fileInfo uploadFileInfo = {0};
    uint32_t uploadStartMs = 0;
    uint32_t uploadEndMs = 0;
    float uploadRate = 0;

    recvBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_RECV_ONCE_BUFFER_SIZE);
    if (recvBuf == NULL) {
        DERROR("[File-Service]  OsdkOsal_Malloc recv buffer error");
        return NULL;
    }

    s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_IDEL;
    s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_IDEL;
    MopPipeline::DataPackType readPack = {(uint8_t *) recvBuf, RELIABLE_RECV_ONCE_BUFFER_SIZE};

    while (sampleFinish == false) {
        DSTATUS("[File-Service]  running file service");
        memset(recvBuf, 0, RELIABLE_RECV_ONCE_BUFFER_SIZE);
        readPack.length = RELIABLE_RECV_ONCE_BUFFER_SIZE;
        mopRet = OM_Pipeline->recvData(readPack, &readPack.length);

        if (mopRet != MOP_PASSED) {
            DERROR("[File-Service]  recv data from cilent error,stat:%lld", mopRet);
            if (mopRet == MOP_CONNECTIONCLOSE) {
                DERROR("[File-Service]  disconnect from cilent now stop task");
                break;
            }
            OsdkOsal_TaskSleepMs(1000);
        } else {
            DSTATUS("[File-Service]  recv data from cilent, len:%d", readPack.length);
            if (readPack.length > 0) {
                sampleProtocolStruct *fileTransfor = (sampleProtocolStruct *) recvBuf;

                switch (fileTransfor->cmd) {
                    case FILE_TRANSFOR_CMD_REQUEST:
                        if (fileTransfor->subcmd == FILE_TRANSFOR_SUBCMD_REQUEST_UPLOAD) {
                            s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_REQUEST_START;
                            s_fileServiceContent.uploadSeqNum = fileTransfor->seq;

                            DSTATUS("[File-Service]  upload request is ok");

                            MD5_Init(&uploadFileMd5Ctx);
                            uploadFileTotalSize = 0;
                            OsdkOsal_GetTimeMs(&uploadStartMs);
                        } else if (fileTransfor->subcmd == FILE_TRANSFOR_SUBCMD_REQUEST_DOWNLOAD) {
                            s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_REQUEST_START;
                            s_fileServiceContent.downloadSeqNum = fileTransfor->seq;

                            DSTATUS("[File-Service]  download request is ok");
                        }
                        break;
                    case FILE_TRANSFOR_CMD_FILE_DOWNLOAD_REQ:
                        DSTATUS("[File-Service]  download request file name:%s",
                                               fileTransfor->data.targetFile.fileName);

                        //TODO: add the file serach logic
                        if (strcmp(fileTransfor->data.targetFile.fileName, "test.mp4") == 0) {
                            s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_FILE_INFO_SUCCESS;
                            s_fileServiceContent.downloadSeqNum = fileTransfor->seq;
                        } else {
                            s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_FILE_INFO_FAILED;
                            s_fileServiceContent.downloadSeqNum = fileTransfor->seq;
                        }
                        break;
                    case FILE_TRANSFOR_CMD_FILE_INFO:
                        DSTATUS(
                            "[File-Service]  upload file info length:%d exist:%d name:%s seq:%d",
                            fileTransfor->data.info.fileLength,
                            fileTransfor->data.info.isExist,
                            fileTransfor->data.info.fileName, fileTransfor->seq);

                        uploadFileInfo.fileLength = fileTransfor->data.info.fileLength;
                        memcpy(uploadFileInfo.md5Buf, fileTransfor->data.info.md5Buf,
                               sizeof(uploadFileInfo.md5Buf));

                        if (uploadFile) fclose(uploadFile);
                        uploadFile = fopen(fileTransfor->data.info.fileName, "wb");
                        if (uploadFile == NULL) {
                            DERROR("[File-Service]  open file error");
                            sampleFinish = true;
                            return NULL;
                        }

                        s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_FILE_INFO_SUCCESS;
                        s_fileServiceContent.uploadSeqNum = fileTransfor->seq;

                        break;
                    case FILE_TRANSFOR_CMD_FILE_DATA:
                        if (uploadFile == NULL) {
                            DERROR("[File-Service]  open file error");
                            sampleFinish = true;
                            return NULL;
                        }

                        if (fileTransfor->subcmd == FILE_TRANSFOR_SUBCMD_FILE_DATA_NORMAL) {
                            s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_DATA_SENDING;
                            s_fileServiceContent.uploadSeqNum = fileTransfor->seq;

                            uploadWriteLen = fwrite(&recvBuf[UTIL_OFFSETOF(sampleProtocolStruct, data)], 1,
                                                    fileTransfor->dataLen, uploadFile);
                            if (uploadWriteLen < 0) {
                                DERROR(
                                    "[File-Service]  upload write normal data to file error, stat:%d.",
                                    uploadWriteLen);
                            } else {
                                uploadFileTotalSize += uploadWriteLen;
                                MD5_Update(&uploadFileMd5Ctx,
                                               &recvBuf[UTIL_OFFSETOF(sampleProtocolStruct, data)],
                                               fileTransfor->dataLen);
                                DSTATUS(
                                    "[File-Service]  upload write data to file success, len:%d  percent:%.1f %%",
                                    uploadWriteLen,
                                    (float) (uploadFileTotalSize * 100) / (float) uploadFileInfo.fileLength);

                            }
                        } else if (fileTransfor->subcmd == FILE_TRANSFOR_SUBCMD_FILE_DATA_END) {
                            s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_DATA_SENDING;
                            s_fileServiceContent.uploadSeqNum = fileTransfor->seq;

                            uploadWriteLen = fwrite(&recvBuf[UTIL_OFFSETOF(sampleProtocolStruct, data)], 1,
                                                    fileTransfor->dataLen, uploadFile);
                            if (uploadWriteLen < 0) {
                                DERROR(
                                    "[File-Service]  upload write end data to file error, stat:%d.",
                                    uploadWriteLen);
                            } else {
                                uploadFileTotalSize += uploadWriteLen;
                                MD5_Update(&uploadFileMd5Ctx,
                                               &recvBuf[UTIL_OFFSETOF(sampleProtocolStruct, data)],
                                               fileTransfor->dataLen);
                                MD5_Final(uploadFileMd5, &uploadFileMd5Ctx);
                                OsdkOsal_GetTimeMs(&uploadEndMs);
                                uploadRate = (float) uploadFileTotalSize * 1000 / (float) (uploadEndMs - uploadStartMs);
                                DSTATUS(
                                    "[File-Service]  upload write data to file success, len:%d  percent:%.1f %%",
                                    uploadWriteLen,
                                    (float) (uploadFileTotalSize * 100) / (float) uploadFileInfo.fileLength);


                                DSTATUS(
                                    "[File-Service]  upload file finished, totalTime:%d ms rate:%.2f Byte/s",
                                     (uploadEndMs - uploadStartMs), uploadRate);
                                fclose(uploadFile);
                                uploadFile = NULL;
                                if (uploadFileInfo.fileLength == uploadFileTotalSize) {
                                    if (memcmp(uploadFileInfo.md5Buf, uploadFileMd5, sizeof(uploadFileMd5)) == 0) {
                                        DERROR(
                                            "[File-Service]  upload file md5 check success");
                                        s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_FINISHED_SUCCESS;
                                        s_fileServiceContent.uploadSeqNum = fileTransfor->seq;
                                    } else {
                                        DERROR(
                                            "[File-Service]  upload file md5 check failed %d %d",
                                            uploadFileInfo.md5Buf[15], uploadFileMd5[15]);
                                        s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_FINISHED_FAILED;
                                        s_fileServiceContent.uploadSeqNum = fileTransfor->seq;
                                    }
                                } else {
                                    DERROR(
                                        "[File-Service]  upload file check file length error");

                                    s_fileServiceContent.uploadState = MOP_FILE_SERVICE_UPLOAD_FINISHED_FAILED;
                                    s_fileServiceContent.uploadSeqNum = fileTransfor->seq;
                                }
                            }
                        }
                        break;
                    case FILE_TRANSFOR_CMD_RESULT:
                        if (fileTransfor->subcmd == FILE_TRANSFOR_SUBCMD_RESULT_OK) {
                            s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_FINISHED_SUCCESS;
                            DERROR("[File-Service]  download file result notify success");
                        } else {
                            s_fileServiceContent.downloadState = MOP_FILE_SERVICE_DOWNLOAD_FINISHED_FAILED;
                            DERROR("[File-Service]  download file result notify failed");
                        }
                        break;
                    default:
                        DSTATUS("[File-Service]  recv the unknown command：0x%02X", fileTransfor->cmd);
                        break;
                }
            }
        }
    }
    OsdkOsal_Free(recvBuf);
    sampleFinish = true;

    return NULL;
}

static void OMDownloadFileTask(MopPipeline *OM_Pipeline)
{
  pthread_t sendTask;
  pthread_t recvTask;
  MopErrCode mopRet;
  int result;

  result = pthread_create(&sendTask, NULL, MopChannelFileServiceSendTask, OM_Pipeline);
  if(result != 0) {
    DERROR("Server send task create failed!\n");
  } else {
    DSTATUS("Server send task create success!\n");
  }

  result = pthread_create(&recvTask, NULL, MopChannelFileServiceRecvTask, OM_Pipeline);
  if(result != 0) {
    DERROR("Server recv task create failed!\n");
  } else {
    DSTATUS("Server recv task create success!\n");
  }

  while(sampleFinish == false) {
    OsdkOsal_TaskSleepMs(500);
  }
}

static void OMUnreliableTransTask(MopPipeline *OM_Pipeline)
{
    uint8_t *sendBuf = NULL;
    uint32_t realLen;
    int returnCode;
    uint32_t sendDataCount = 0;
    MopErrCode mopRet;

    sendBuf = (uint8_t *)OsdkOsal_Malloc(UNRELIABLE_SEND_ONCE_BUFFER_SIZE);
    if (sendBuf == NULL) {
        DERROR("OsdkOsal_Malloc send buffer error");
        sampleFinish = true;
        return;
    }
    MopPipeline::DataPackType reqPack = {.data = (uint8_t *) sendBuf, .length = UNRELIABLE_SEND_ONCE_BUFFER_SIZE};

    OsdkOsal_TaskSleepMs(10000);
    while (1) {
        sendDataCount++;
        memset(sendBuf, sendDataCount, UNRELIABLE_SEND_ONCE_BUFFER_SIZE);
        reqPack.length = UNRELIABLE_SEND_ONCE_BUFFER_SIZE;
        mopRet = OM_Pipeline->sendData(reqPack, &reqPack.length);
        ASSERT_MOP_RET(mopRet)
        total_len += reqPack.length;
        OsdkOsal_TaskSleepMs(1000 / TEST_OM_UNRELIABLE_TRANSFOR_FREQ);
    }
    OsdkOsal_Free(sendBuf);
}

static void* MopServerTask(void *arg)
{
  Vehicle *vehicle = (Vehicle *)arg;
  PipelineType type = UNRELIABLE;
  PipelineID id = TEST_OM_UNRELIABLE_PIPELINE_ID;

  /*! create mop server */
  MopServer *server = new MopServer();

  /*! connect pipeline */
  MopPipeline *OM_Pipeline = NULL;
  if (isReliable) {
    type = RELIABLE;
    id = TEST_OM_RELIABLE_PIPELINE_ID;
  }
  if ((server->accept(id, type, OM_Pipeline)
      != MOP_PASSED) || (OM_Pipeline == NULL)) {
    DERROR("MOP server accept failed");
    delete server;
    return NULL;
  } else {
    DSTATUS("accept successfully");
  }
  isAccept = true;

  /*! OSDK download file from  */
  if (isReliable) {
    OMDownloadFileTask(OM_Pipeline);
  } else {
    OMUnreliableTransTask(OM_Pipeline);
  }

  /*! Disconnect pipeline */
  if (server->close(id) != MOP_PASSED) {
    DERROR("MOP Pipeline disconnect pipeline(%d) failed", id);
  } else {
    DSTATUS("Disconnect mop pipeline id(%d) successfully", id);
  }
  sampleFinish = true;
  delete server;

  return NULL;
}

int main(int argc, char** argv)
{
  int result;
  pthread_t serverTask;
  char input = 0;
  uint32_t time = 0;

  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();

  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }

  cout << "OSDK<--->MSDK MOP Sample!\n"
       << "please choose connection type!\n"
       << "a: reliable connection\n"
       << "b: unreliable connection" << endl;
  cin >> input;

  switch(input)
  {
    case 'a':
      isReliable = true; break;
    case 'b':
      isReliable = false; break;
    default:
      cout << "invalid input!";
      return 1;
  }

  result = pthread_create(&serverTask, NULL, MopServerTask, vehicle);
  if(result != 0) {
    DERROR("Server task create failed!\n");
  } else {
    DSTATUS("Server task create success!\n");
  }

  //press q to exit
  while(!sampleFinish) {

    if(!isReliable && isAccept) {
        time++;
        DSTATUS("total send len: %d\n", total_len);
        DSTATUS("total time: %d s\n", time);
        DSTATUS("average speed: %d KBps\n", (total_len/1024)/time);
    }
    OsdkOsal_TaskSleepMs(1000);
  }
  DSTATUS("Sample exit.");
}
