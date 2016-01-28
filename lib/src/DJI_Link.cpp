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

void DJI::onboardSDK::CoreAPI::sendData(unsigned char *buf)
{
    size_t ans;
    Header *pHeader = (Header *)buf;
    ans = driver->send(buf, pHeader->length);
    if (ans == 0)
        API_LOG(driver, STATUS_LOG, "Port not send");
    if (ans == (size_t)-1)
        API_LOG(driver, ERROR_LOG, "Port closed");
}

void DJI::onboardSDK::CoreAPI::appHandler(Header *header)
{
    Header *p2header;
    CallBack callBack = 0;
    if (header->isAck == 1)
    {
        if (header->sessionID > 1 && header->sessionID < 32)
        {
            if (CMDSessionTab[header->sessionID].usageFlag == 1)
            {
                driver->lockMemory();
                p2header = (Header *)CMDSessionTab[header->sessionID].mmu->pmem;
                if (p2header->sessionID == header->sessionID &&
                    p2header->sequenceNumber == header->sequenceNumber)
                {
                    API_LOG(driver, DEBUG_LOG, "Recv Session %d ACK\n", p2header->sessionID);

                    callBack = CMDSessionTab[header->sessionID].handler;
                    freeSession(&CMDSessionTab[header->sessionID]);
                    driver->freeMemory();
                    if (callBack)
                        //! @todo new algorithm call in a thread
                        callBack(this, header, CMDSessionTab[header->sessionID].userData);
                }
                else
                    driver->freeMemory();
            }
        }
    }
    else
    {
        switch (header->sessionID)
        {
            case 0:
                recvReqData(header);
                break;
            case 1:
            //! @todo unnecessary ack in case 1. Maybe add code later
            //! @todo check algorithm
            default: //! @note session id is 2
                API_LOG(driver, STATUS_LOG, "ACK %d", header->sessionID);

                if (ACKSessionTab[header->sessionID - 1].sessionStatus == ACK_SESSION_PROCESS)
                {
                    API_LOG(driver, DEBUG_LOG, "This session is waiting for App ack:"
                                               "session id=%d,seq_num=%d\n",
                            header->sessionID, header->sequenceNumber);
                }
                else if (ACKSessionTab[header->sessionID - 1].sessionStatus == ACK_SESSION_IDLE)
                {
                    if (header->sessionID > 1)
                        ACKSessionTab[header->sessionID - 1].sessionStatus =
                            ACK_SESSION_PROCESS;
                    recvReqData(header);
                }
                else if (ACKSessionTab[header->sessionID - 1].sessionStatus ==
                         ACK_SESSION_USING)
                {
                    driver->lockMemory();
                    p2header = (Header *)ACKSessionTab[header->sessionID - 1].mmu->pmem;
                    if (p2header->sequenceNumber == header->sequenceNumber)
                    {
                        API_LOG(driver, DEBUG_LOG, "repeat ACK to remote,session "
                                                   "id=%d,seq_num=%d\n",
                                header->sessionID, header->sequenceNumber);
                        sendData(ACKSessionTab[header->sessionID - 1].mmu->pmem);
                        driver->freeMemory();
                    }
                    else
                    {
                        API_LOG(driver, DEBUG_LOG,
                                "same session,but new seq_num pkg,session id=%d,"
                                "pre seq_num=%d,cur seq_num=%d\n",
                                header->sessionID, p2header->sequenceNumber,
                                header->sequenceNumber);
                        ACKSessionTab[header->sessionID - 1].sessionStatus =
                            ACK_SESSION_PROCESS;
                        driver->freeMemory();
                        recvReqData(header);
                    }
                }
                break;
        }
    }
}

void DJI::onboardSDK::CoreAPI::sendPoll()
{
    unsigned char i;
    time_ms curTimestamp;
    for (i = 1; i < SESSION_TABLE_NUM; i++)
    {
        if (CMDSessionTab[i].usageFlag == 1)
        {
            curTimestamp = driver->getTimeStamp();
            if ((curTimestamp - CMDSessionTab[i].preTimestamp) > CMDSessionTab[i].timeout)
            {
                driver->lockMemory();
                if (CMDSessionTab[i].retry > 0)
                {
                    if (CMDSessionTab[i].sent >= CMDSessionTab[i].retry)
                    {
                        API_LOG(driver, DEBUG_LOG, "Free ssion %d\n",
                                CMDSessionTab[i].sessionID);

                        freeSession(&CMDSessionTab[i]);
                    }
                    else
                    {
                        API_LOG(driver, DEBUG_LOG, "Retry session %d\n",
                                CMDSessionTab[i].sessionID);
                        sendData(CMDSessionTab[i].mmu->pmem);
                        CMDSessionTab[i].preTimestamp = curTimestamp;
                        CMDSessionTab[i].sent++;
                    }
                }
                else
                {
                    API_LOG(driver, DEBUG_LOG, "send once %d\n", i);
                    sendData(CMDSessionTab[i].mmu->pmem);
                    CMDSessionTab[i].preTimestamp = curTimestamp;
                }
                driver->freeMemory();
            }
            else
            {
                API_LOG(driver, DEBUG_LOG, "timeout Session: %d \n", i);
            }
        }
    }
}

void DJI::onboardSDK::CoreAPI::readPoll()
{
    int read_len;
    uint8_t buf[BUFFER_SIZE];
    read_len = driver->readall(buf, BUFFER_SIZE);
#ifdef API_BUFFER_DATA
    onceRead = read_len;
    totalRead += onceRead;
#endif // API_BUFFER_DATA
    for (int i = 0; i < read_len; i++)
    {
        byteHandler(buf[i]);
    }
}

void CoreAPI::callbackPoll()
{
    //! @todo implement callbackPoll
    if (cblistTail != CALLBACK_LIST_NUM)
    {
    }
}

void DJI::onboardSDK::CoreAPI::setup()
{
    setupMMU();
    setupSession();
}

void DJI::onboardSDK::CoreAPI::setKey(const char *key)
{
    transformTwoByte(key, filter.sdkKey);
    filter.encode = 1;
}

void CoreAPI::setActivation(bool isActivated)
{
    if (isActivated)
        broadcastData.activation = 1;
    else
        broadcastData.activation = 0;
}

void CoreAPI::setSyncFreq(uint32_t freqInHz)
{
    send(0, 1, SET_SYNC, CODE_SYNC_BROADCAST, &freqInHz, sizeof(freqInHz));
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

int DJI::onboardSDK::CoreAPI::ackInterface(Ack *parameter)
{
    unsigned short ret = 0;
    ACKSession *ack_session = (ACKSession *)NULL;

    if (parameter->length > PRO_PURE_DATA_MAX_SIZE)
    {
        API_LOG(driver, ERROR_LOG, "length=%d is oversize\n", parameter->length);
        return -1;
    }

    if (parameter->sessionID == 0)
    {
        //! @note nothing todo session 0 is a nack session.
        return 0;
    }
    else if (parameter->sessionID > 0 && parameter->sessionID < 32)
    {
        driver->lockMemory();
        ack_session = allocACK(parameter->sessionID,
                               calculateLength(parameter->length, parameter->encrypt));
        if (ack_session == (ACKSession *)NULL)
        {
            API_LOG(driver, ERROR_LOG, "there is not enough memory\n");
            driver->freeMemory();
            return -1;
        }

        ret = encrypt(ack_session->mmu->pmem, parameter->buf, parameter->length, 1,
                      parameter->encrypt, parameter->sessionID, parameter->seqNum);
        if (ret == 0)
        {
            API_LOG(driver, ERROR_LOG, "encrypt ERROR\n");
            driver->freeMemory();
            return -1;
        }

        API_LOG(driver, DEBUG_LOG, "Sending data!");
        sendData(ack_session->mmu->pmem);
        driver->freeMemory();
        ack_session->sessionStatus = ACK_SESSION_USING;
        return 0;
    }

    return -1;
}

int DJI::onboardSDK::CoreAPI::sendInterface(Command *parameter)
{
    unsigned short ret = 0;
    CMDSession *cmd_session = (CMDSession *)NULL;
    if (parameter->length > PRO_PURE_DATA_MAX_SIZE)
    {
        API_LOG(driver, ERROR_LOG, "ERROR,length=%d is oversize\n", parameter->length);
        return -1;
    }

    switch (parameter->sessionMode)
    {
        case 0:
            driver->lockMemory();
            cmd_session = allocSession(CMD_SESSION_0,
                                       calculateLength(parameter->length, parameter->encrypt));

            if (cmd_session == (CMDSession *)NULL)
            {
                driver->freeMemory();
                API_LOG(driver, ERROR_LOG, "ERROR,there is not enough memory\n");
                return -1;
            }
            ret = encrypt(cmd_session->mmu->pmem, parameter->buf, parameter->length, 0,
                          parameter->encrypt, cmd_session->sessionID, seq_num);
            if (ret == 0)
            {
                API_LOG(driver, ERROR_LOG, "encrypt ERROR\n");
                freeSession(cmd_session);
                driver->freeMemory();
                return -1;
            }

            API_LOG(driver, DEBUG_LOG, "send data in session mode 0\n");

            sendData(cmd_session->mmu->pmem);
            seq_num++;
            freeSession(cmd_session);
            driver->freeMemory();
            break;
        case 1:
            driver->lockMemory();
            cmd_session = allocSession(CMD_SESSION_1,
                                       calculateLength(parameter->length, parameter->encrypt));
            if (cmd_session == (CMDSession *)NULL)
            {
                driver->freeMemory();
                API_LOG(driver, ERROR_LOG, "ERROR,there are not enough memory\n");
                return -1;
            }
            if (seq_num == cmd_session->preSeqNum)
                seq_num++;
            ret = encrypt(cmd_session->mmu->pmem, parameter->buf, parameter->length, 0,
                          parameter->encrypt, cmd_session->sessionID, seq_num);
            if (ret == 0)
            {
                API_LOG(driver, ERROR_LOG, "encrypt ERROR\n");
                freeSession(cmd_session);
                driver->freeMemory();
                return -1;
            }
            cmd_session->preSeqNum = seq_num++;

            cmd_session->handler = parameter->handler;
            cmd_session->userData = parameter->userData;
            cmd_session->timeout =
                (parameter->timeout > POLL_TICK) ? parameter->timeout : POLL_TICK;
            cmd_session->preTimestamp = driver->getTimeStamp();
            cmd_session->sent = 1;
            cmd_session->retry = 1;
            API_LOG(driver, DEBUG_LOG, "sending session %d\n", cmd_session->sessionID);
            sendData(cmd_session->mmu->pmem);
            driver->freeMemory();
            break;
        case 2:
            driver->lockMemory();
            cmd_session = allocSession(CMD_SESSION_AUTO,
                                       calculateLength(parameter->length, parameter->encrypt));
            if (cmd_session == (CMDSession *)NULL)
            {
                driver->freeMemory();
                API_LOG(driver, ERROR_LOG, "ERROR,there is not enough memory\n");
                return -1;
            }
            if (seq_num == cmd_session->preSeqNum)
            {
                seq_num++;
            }
            ret = encrypt(cmd_session->mmu->pmem, parameter->buf, parameter->length, 0,
                          parameter->encrypt, cmd_session->sessionID, seq_num);

            if (ret == 0)
            {
                API_LOG(driver, ERROR_LOG, "encrypt ERROR");
                freeSession(cmd_session);
                driver->freeMemory();
                return -1;
            }
            cmd_session->preSeqNum = seq_num++;
            cmd_session->handler = parameter->handler;
            cmd_session->userData = parameter->userData;
            cmd_session->timeout =
                (parameter->timeout > POLL_TICK) ? parameter->timeout : POLL_TICK;
            cmd_session->preTimestamp = driver->getTimeStamp();
            cmd_session->sent = 1;
            cmd_session->retry = parameter->retry;
            API_LOG(driver, DEBUG_LOG, "Sending session %d\n", cmd_session->sessionID);
            sendData(cmd_session->mmu->pmem);
            driver->freeMemory();
            break;
        default:
            API_LOG(driver, ERROR_LOG, "Unknown mode:%d\n", parameter->sessionMode);
            break;
    }
    return 0;
}
