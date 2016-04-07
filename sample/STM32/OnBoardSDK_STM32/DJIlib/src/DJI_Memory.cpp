/*! @brief
 *  @file DJI_Memory.cpp
 *  @version V2.0
 *  @date Nov 11, 2015
 *  @author wuyunwei,william.wu
 *
 *  @abstract
 *  This file mainly implement fuctions in DJI_API.h
 *
 *  All Functions in this file is private function,
 *  which is used for memory and session management.
 *
 *  @attention
 *  It is not necessary to include DJI_link.h in any custom code file.
 *  All functions in this file are not API function.
 *  Do not modify this file, if you are not sure about it.
 *  Created on: 24 Aug, 2015
 *      Author: wuyuwei
 *  Modified on: Nov 11, 2015
 *  by william.wu
 */

#include <stdio.h>
#include <string.h>
#include "DJI_Memory.h"
#include "DJI_API.h"

using namespace DJI::onboardSDK;

void DJI::onboardSDK::CoreAPI::setupMMU()
{
    unsigned int i;
    MMU[0].tabIndex = 0;
    MMU[0].usageFlag = 1;
    MMU[0].pmem = memory;
    MMU[0].memSize = 0;
    for (i = 1; i < (MMU_TABLE_NUM - 1); i++)
    {
        MMU[i].tabIndex = i;
        MMU[i].usageFlag = 0;
    }
    MMU[MMU_TABLE_NUM - 1].tabIndex = MMU_TABLE_NUM - 1;
    MMU[MMU_TABLE_NUM - 1].usageFlag = 1;
    MMU[MMU_TABLE_NUM - 1].pmem = memory + MEMORY_SIZE;
    MMU[MMU_TABLE_NUM - 1].memSize = 0;
}

void freeMemory(MMU_Tab *mmu_tab)
{
    if (mmu_tab == (MMU_Tab *)0)
        return;
    if (mmu_tab->tabIndex == 0 || mmu_tab->tabIndex == (MMU_TABLE_NUM - 1))
        return;
    mmu_tab->usageFlag = 0;
}

MMU_Tab *DJI::onboardSDK::CoreAPI::allocMemory(unsigned short size)
{
    unsigned int mem_used = 0;
    unsigned char i;
    unsigned char j = 0;
    unsigned char mmu_tab_used_num = 0;
    unsigned char mmu_tab_used_index[MMU_TABLE_NUM];

    unsigned int temp32;
    unsigned int temp_area[2] = { 0xFFFFFFFF, 0xFFFFFFFF };

    unsigned int record_temp32 = 0;
    unsigned char magic_flag = 0;

    if (size > PRO_PURE_DATA_MAX_SIZE || size > MEMORY_SIZE)
        return (MMU_Tab *)0;

    for (i = 0; i < MMU_TABLE_NUM; i++)
        if (MMU[i].usageFlag == 1)
        {
            mem_used += MMU[i].memSize;
            mmu_tab_used_index[mmu_tab_used_num++] = MMU[i].tabIndex;
        }

    if (MEMORY_SIZE < (mem_used + size))
        return (MMU_Tab *)0;

    if (mem_used == 0)
    {
        MMU[1].pmem = MMU[0].pmem;
        MMU[1].memSize = size;
        MMU[1].usageFlag = 1;
        return &MMU[1];
    }

    for (i = 0; i < (mmu_tab_used_num - 1); i++)
        for (j = 0; j < (mmu_tab_used_num - i - 1); j++)
            if (MMU[mmu_tab_used_index[j]].pmem > MMU[mmu_tab_used_index[j + 1]].pmem)
            {
                mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
                mmu_tab_used_index[j] ^= mmu_tab_used_index[j + 1];
                mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
            }

    for (i = 0; i < (mmu_tab_used_num - 1); i++)
    {
        temp32 = (unsigned int)(MMU[mmu_tab_used_index[i + 1]].pmem -
                                MMU[mmu_tab_used_index[i]].pmem);

        if ((temp32 - MMU[mmu_tab_used_index[i]].memSize) >= size)
        {
            if (temp_area[1] > (temp32 - MMU[mmu_tab_used_index[i]].memSize))
            {
                temp_area[0] = MMU[mmu_tab_used_index[i]].tabIndex;
                temp_area[1] = temp32 - MMU[mmu_tab_used_index[i]].memSize;
            }
        }

        record_temp32 += temp32 - MMU[mmu_tab_used_index[i]].memSize;
        if (record_temp32 >= size && magic_flag == 0)
        {
            j = i;
            magic_flag = 1;
        }
    }

    if (temp_area[0] == 0xFFFFFFFF && temp_area[1] == 0xFFFFFFFF)
    {
        for (i = 0; i < j; i++)
        {
            if (MMU[mmu_tab_used_index[i + 1]].pmem >
                (MMU[mmu_tab_used_index[i]].pmem + MMU[mmu_tab_used_index[i]].memSize))
            {
                memmove(MMU[mmu_tab_used_index[i]].pmem + MMU[mmu_tab_used_index[i]].memSize,
                        MMU[mmu_tab_used_index[i + 1]].pmem,
                        MMU[mmu_tab_used_index[i + 1]].memSize);
                MMU[mmu_tab_used_index[i + 1]].pmem =
                    MMU[mmu_tab_used_index[i]].pmem + MMU[mmu_tab_used_index[i]].memSize;
            }
        }

        for (i = 1; i < (MMU_TABLE_NUM - 1); i++)
        {
            if (MMU[i].usageFlag == 0)
            {
                MMU[i].pmem =
                    MMU[mmu_tab_used_index[j]].pmem + MMU[mmu_tab_used_index[j]].memSize;

                MMU[i].memSize = size;
                MMU[i].usageFlag = 1;
                return &MMU[i];
            }
        }
        return (MMU_Tab *)0;
    }

    for (i = 1; i < (MMU_TABLE_NUM - 1); i++)
    {
        if (MMU[i].usageFlag == 0)
        {
            MMU[i].pmem = MMU[temp_area[0]].pmem + MMU[temp_area[0]].memSize;

            MMU[i].memSize = size;
            MMU[i].usageFlag = 1;
            return &MMU[i];
        }
    }

    return (MMU_Tab *)0;
}

void DJI::onboardSDK::CoreAPI::setupSession()
{
    unsigned int i;
    for (i = 0; i < SESSION_TABLE_NUM; i++)
    {
        CMDSessionTab[i].sessionID = i;
        CMDSessionTab[i].usageFlag = 0;
        CMDSessionTab[i].mmu = (MMU_Tab *)NULL;
    }

    for (i = 0; i < (SESSION_TABLE_NUM - 1); i++)
    {
        ACKSessionTab[i].sessionID = i + 1;
        ACKSessionTab[i].sessionStatus = ACK_SESSION_IDLE;
        ACKSessionTab[i].mmu = (MMU_Tab *)NULL;
    }
}

/*! @note Alloc a cmd session for sending cmd data
 *  when arg session_id = 0/1, it means select session 0/1 to send cmd
 *  otherwise set arg session_id = CMD_SESSION_AUTO (32), which means auto
 *  select a idle session id is between 2~31.
 */

CMDSession *DJI::onboardSDK::CoreAPI::allocSession(unsigned short session_id,
                                                   unsigned short size)
{
    unsigned int i;
    API_LOG(driver, DEBUG_LOG, "Alloc size %d", size);
    MMU_Tab *mmu = NULL;

    if (session_id == 0 || session_id == 1)
    {
        if (this->CMDSessionTab[session_id].usageFlag == 0)
            i = session_id;
        else
        {
            /* session is busy */
            API_LOG(driver, ERROR_LOG, "session %d is busy\n", session_id);
            return NULL;
        }
    }
    else
    {
        for (i = 2; i < SESSION_TABLE_NUM; i++)
            if (CMDSessionTab[i].usageFlag == 0)
                break;
    }
    if (i < 32 && CMDSessionTab[i].usageFlag == 0)
    {
        CMDSessionTab[i].usageFlag = 1;
        mmu = allocMemory(size);
        if (mmu == NULL)
            CMDSessionTab[i].usageFlag = 0;
        else
        {
            CMDSessionTab[i].mmu = mmu;
            return &CMDSessionTab[i];
        }
    }
    return NULL;
}

void DJI::onboardSDK::CoreAPI::freeSession(CMDSession *session)
{
    if (session->usageFlag == 1)
    {
        API_LOG(driver, DEBUG_LOG, "session id %d\n", session->sessionID);
        freeMemory(session->mmu);
        session->usageFlag = 0;
    }
}

ACKSession *DJI::onboardSDK::CoreAPI::allocACK(unsigned short session_id, unsigned short size)
{
    MMU_Tab *mmu = NULL;
    if (session_id > 0 && session_id < 32)
    {
        if (ACKSessionTab[session_id - 1].mmu)
            freeACK(&ACKSessionTab[session_id - 1]);
        mmu = allocMemory(size);
        if (mmu == NULL)
        {
            //! @todo optmize
        }
        else
        {
            ACKSessionTab[session_id - 1].mmu = mmu;
            return &ACKSessionTab[session_id - 1];
        }
    }
    return NULL;
}

void DJI::onboardSDK::CoreAPI::freeACK(ACKSession *session) { freeMemory(session->mmu); }
