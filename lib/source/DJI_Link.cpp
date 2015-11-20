/*! @brief
 *  @file DJI_Link.cpp
 *  @version V2.0
 *  @date Nov 11, 2015
 *  @author wuyuwei, william.wu
 *
 *  @abstract
 *  This file mainly implement functions in DJI_API.h
 *
 *  All Functions in this file is private function,
 *  which is used for decode data and build up data link.
 *
 *  Definitions in DJI_Link.h are private variables.
 *
 *  @attention
 *  It is not necessary to include DJI_link.h in any custom code file.
 *  All functions in this file are not API function.
 *  Do not modify this file, if you are not sure about it.
 *
 *  @version features:
 *  -* @version V2.0
 *  -* @date Nov 11, 2015
 *  -* DJI-onboard-SDK Object-Oriented implementation.
 *  -* @version V1.0
 *  -* @date Mar 12, 2015
 *  -* DJI-onboard-SDK c-like implementation.
 */

#include <stdio.h>
#include <string.h>
#include "DJI_Link.h"
#include "DJI_Codec.h"
#include "DJI_API.h"

using namespace DJI::onboardSDK;

void DJI::onboardSDK::API::sendData(unsigned char *buf)
{
    Header *pHeader = (Header *)buf;
    driver->send(buf, pHeader->length);
}

void DJI::onboardSDK::API::appHandler(Header *header)
{
    Header *p2header;
    CallBack callBack = 0;
    if (header->is_ack == 1)
    {
        if (header->sessionID > 1 && header->sessionID < 32)
        {
            if (CMDSessionTab[header->sessionID].usageFlag == 1)
            {
                driver->lockMemory();
                p2header = (Header *)CMDSessionTab[header->sessionID].mmu->pmem;
                if (p2header->sessionID == header->sessionID &&
                    p2header->sequence_number == header->sequence_number)
                {
                    API_DEBUG("Recv Session %d ACK\n", p2header->sessionID);
                    callBack = CMDSessionTab[header->sessionID].callback;
                    freeSession(&CMDSessionTab[header->sessionID]);
                    driver->freeMemory();
                    if (callBack)
                        callBack(this, header);
                }
                else
                    driver->freeMemory();
            }
        }
    }
    else
    {
        // TODO,is a request package

        switch (header->sessionID)
        {
            case 0:
                recvReqData(header);
                break;
            case 1:
            default:
                if (ACKSessionTab[header->sessionID - 1].session_status ==
                    ACK_SESSION_PROCESS)
                {
                    API_DEBUG("This session is waiting for App ack:"
                              "session id=%d,seq_num=%d\n",
                              header->sessionID, header->sequence_number);
                }
                else if (ACKSessionTab[header->sessionID - 1].session_status ==
                         ACK_SESSION_IDLE)
                {
                    if (header->sessionID > 1)
                        ACKSessionTab[header->sessionID - 1].session_status =
                            ACK_SESSION_PROCESS;
                    recvReqData(header);
                }
                else if (ACKSessionTab[header->sessionID - 1].session_status ==
                         ACK_SESSION_USING)
                {
                    driver->lockMemory();
                    p2header =
                        (Header *)
                            ACKSessionTab[header->sessionID - 1].mmu->pmem;
                    if (p2header->sequence_number == header->sequence_number)
                    {
                        API_DEBUG("repeat ACK to remote,session "
                                  "id=%d,seq_num=%d\n",
                                  header->sessionID, header->sequence_number);
                        sendData(
                            ACKSessionTab[header->sessionID - 1].mmu->pmem);
                        driver->freeMemory();
                    }
                    else
                    {
                        API_DEBUG(
                            "same session,but new seq_num pkg,session id=%d,"
                            "pre seq_num=%d,cur seq_num=%d\n",
                            header->sessionID, p2header->sequence_number,
                            header->sequence_number);
                        ACKSessionTab[header->sessionID - 1].session_status =
                            ACK_SESSION_PROCESS;
                        driver->freeMemory();
                        recvReqData(header);
                    }
                }
                break;
        }
    }
}

void DJI::onboardSDK::API::sendPoll()
{
    unsigned char i;
    unsigned int cur_timestamp;
    for (i = 1; i < SESSION_TABLE_NUM; i++)
    {
        if (CMDSessionTab[i].usageFlag == 1)
        {
            cur_timestamp = driver->getTimeStamp();
            if ((cur_timestamp - CMDSessionTab[i].pre_timestamp) >
                CMDSessionTab[i].timeout)
            {
                driver->lockMemory();
                if (CMDSessionTab[i].retry > 0)
                {
                    if (CMDSessionTab[i].sent >= CMDSessionTab[i].retry)
                    {
                        API_DEBUG("Free ssion %d\n",
                                  CMDSessionTab[i].sessionID);
                        freeSession(&CMDSessionTab[i]);
                    }
                    else
                    {
                        API_DEBUG("Retry session %d\n",
                                  CMDSessionTab[i].sessionID);
                        sendData(CMDSessionTab[i].mmu->pmem);
                        CMDSessionTab[i].pre_timestamp = cur_timestamp;
                        CMDSessionTab[i].sent++;
                    }
                }
                else
                {
                    API_DEBUG("send once %d", i);
                    sendData(CMDSessionTab[i].mmu->pmem);
                    CMDSessionTab[i].pre_timestamp = cur_timestamp;
                }
                driver->freeMemory();
            }
        }
    }
}

void DJI::onboardSDK::API::readPoll()
{
    int read_len;
    uint8_t buf[BUFFER_SIZE];
    read_len = driver->readall(buf, BUFFER_SIZE);
    for (int i = 0; i < read_len; i++) byteHandler(buf[i]);
}

void DJI::onboardSDK::API::setup()
{
    setupMMU();
    setupSession();
}

void DJI::onboardSDK::API::setKey(const char *key)
{
    transformTwoByte(key, filter.comm_key);
    filter.enc_enabled = 1;
}

unsigned short calculateLength(unsigned short size, unsigned short encrypt_flag)
{
    unsigned short len;
    if (encrypt_flag)
        len = size + sizeof(Header) + 4 + (16 - size % 16);
    else
        len = size + sizeof(Header) + 4;
    return len;
}

int DJI::onboardSDK::API::ackInterface(Ack *parameter)
{
    unsigned short ret = 0;
    ACKSession *ack_session = (ACKSession *)NULL;

    if (parameter->length > PRO_PURE_DATA_MAX_SIZE)
    {
        API_ERROR("length=%d is oversize\n", parameter->length);
        return -1;
    }

    if (parameter->session_id == 0)
    {
        //! @todo
        ;
    }
    else if (parameter->session_id > 0 && parameter->session_id < 32)
    {
        driver->lockMemory();
        ack_session = allocACK(
            parameter->session_id,
            calculateLength(parameter->length, parameter->need_encrypt));
        if (ack_session == (ACKSession *)NULL)
        {
            API_ERROR("there is not enough memory\n");
            driver->freeMemory();
            return -1;
        }

        ret = encrypt(ack_session->mmu->pmem, parameter->buf, parameter->length,
                      1, parameter->need_encrypt, parameter->session_id,
                      parameter->seq_num);
        if (ret == 0)
        {
            API_ERROR("encrypt ERROR\n");
            driver->freeMemory();
            return -1;
        }

        sendData(ack_session->mmu->pmem);
        driver->freeMemory();
        ack_session->session_status = ACK_SESSION_USING;
        return 0;
    }

    return -1;
}

int DJI::onboardSDK::API::sendInterface(Command *parameter)
{
    unsigned short ret = 0;
    CMDSession *cmd_session = (CMDSession *)NULL;
    if (parameter->length > PRO_PURE_DATA_MAX_SIZE)
    {
        API_ERROR("ERROR,length=%d is oversize\n", parameter->length);
        return -1;
    }

    switch (parameter->session_mode)
    {
        case 0:
            driver->lockMemory();
            cmd_session = allocSession(
                CMD_SESSION_0,
                calculateLength(parameter->length, parameter->need_encrypt));
            if (cmd_session == (CMDSession *)NULL)
            {
                driver->freeMemory();
                API_ERROR("ERROR,there is not enough memory\n");
                return -1;
            }
            ret = encrypt(cmd_session->mmu->pmem, parameter->buf,
                          parameter->length, 0, parameter->need_encrypt,
                          cmd_session->sessionID, seq_num);
            if (ret == 0)
            {
                API_ERROR("encrypt ERROR\n");
                freeSession(cmd_session);
                driver->freeMemory();
                return -1;
            }

            API_DEBUG("send data in session mode 0\n");

            sendData(cmd_session->mmu->pmem);
            seq_num++;
            freeSession(cmd_session);
            driver->freeMemory();
            break;
        case 1:
            driver->lockMemory();
            cmd_session = allocSession(
                CMD_SESSION_1,
                calculateLength(parameter->length, parameter->need_encrypt));
            if (cmd_session == (CMDSession *)NULL)
            {
                driver->freeMemory();
                API_ERROR("ERROR,there are not enough memory\n");
                return -1;
            }
            if (seq_num == cmd_session->pre_seq_num)
                seq_num++;
            ret = encrypt(cmd_session->mmu->pmem, parameter->buf,
                          parameter->length, 0, parameter->need_encrypt,
                          cmd_session->sessionID, seq_num);
            if (ret == 0)
            {
                API_ERROR("encrypt ERROR\n");
                freeSession(cmd_session);
                driver->freeMemory();
                return -1;
            }
            cmd_session->pre_seq_num = seq_num++;
            cmd_session->callback = parameter->callback;
            cmd_session->timeout = (parameter->timeout > POLL_TICK)
                                       ? parameter->timeout
                                       : POLL_TICK;
            cmd_session->pre_timestamp = driver->getTimeStamp();
            cmd_session->sent = 1;
            cmd_session->retry = 1;
            API_DEBUG("sending session %d\n", cmd_session->sessionID);
            sendData(cmd_session->mmu->pmem);
            driver->freeMemory();
            break;
        case 2:
            driver->lockMemory();
            cmd_session = allocSession(
                CMD_SESSION_AUTO,
                calculateLength(parameter->length, parameter->need_encrypt));
            if (cmd_session == (CMDSession *)NULL)
            {
                driver->freeMemory();
                API_ERROR("ERROR,there is not enough memory\n");
                return -1;
            }
            if (seq_num == cmd_session->pre_seq_num)
                seq_num++;
            ret = encrypt(cmd_session->mmu->pmem, parameter->buf,
                          parameter->length, 0, parameter->need_encrypt,
                          cmd_session->sessionID, seq_num);
            if (ret == 0)
            {
                API_ERROR("encrypt ERROR\n");
                freeSession(cmd_session);
                driver->freeMemory();
                return -1;
            }
            cmd_session->pre_seq_num = seq_num++;
            cmd_session->callback = parameter->callback;
            cmd_session->timeout = (parameter->timeout > POLL_TICK)
                                       ? parameter->timeout
                                       : POLL_TICK;
            cmd_session->pre_timestamp = driver->getTimeStamp();
            cmd_session->sent = 1;
            cmd_session->retry = parameter->retry_time;
            API_DEBUG("Sending session %d\n", cmd_session->sessionID);
            sendData(cmd_session->mmu->pmem);
            driver->freeMemory();
            break;
        default:
            API_ERROR("Unknown mode:%d\n", parameter->session_mode);
            break;
    }
    return 0;
}
