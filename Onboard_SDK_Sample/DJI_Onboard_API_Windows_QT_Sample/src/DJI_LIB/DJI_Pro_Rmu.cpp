/*
 * DJI_Pro_Rmu.cpp
 * Des:RMU,means Resource Management Unit, includes memory and session management
 *  Created on: 24 Aug, 2015
 *      Author: wuyuwei
 */


#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "DJI_Pro_Rmu.h"

static pthread_mutex_t mmu_lock = PTHREAD_MUTEX_INITIALIZER;

static MMU_Tab DJI_MMU_Tab[MMU_TABLE_NUM];
static unsigned char Static_Memory[STATIC_MEMORY_SIZE];

static CMD_Session_Tab DJI_CMD_Session_Tab[SESSION_TABLE_NUM];
static ACK_Session_Tab DJI_ACK_Session_Tab[SESSION_TABLE_NUM - 1]; //session 0 is a nak session id

CMD_Session_Tab * Get_CMD_Session_Tab(void)
{
	return DJI_CMD_Session_Tab;
}

ACK_Session_Tab * Get_ACK_Session_Tab(void)
{
	return DJI_ACK_Session_Tab;
}

void MMU_Setup(void)
{
	int i;
	DJI_MMU_Tab[0].tab_index = 0;
	DJI_MMU_Tab[0].usage_flag = 1;
	DJI_MMU_Tab[0].pmem = Static_Memory;
	DJI_MMU_Tab[0].mem_size = 0;
	for(i = 1 ; i < (MMU_TABLE_NUM - 1) ; i ++)
	{
		DJI_MMU_Tab[i].tab_index = i;
		DJI_MMU_Tab[i].usage_flag = 0;
	}
	DJI_MMU_Tab[MMU_TABLE_NUM - 1].tab_index = MMU_TABLE_NUM - 1;
	DJI_MMU_Tab[MMU_TABLE_NUM - 1].usage_flag = 1;
	DJI_MMU_Tab[MMU_TABLE_NUM - 1].pmem = Static_Memory + STATIC_MEMORY_SIZE;
	DJI_MMU_Tab[MMU_TABLE_NUM - 1].mem_size = 0;
}

void Get_Memory_Lock(void)
{
	pthread_mutex_lock(&mmu_lock);
}

void Free_Memory_Lock(void)
{
	pthread_mutex_unlock(&mmu_lock);
}

void Free_Memory(MMU_Tab *mmu_tab)
{
	if(mmu_tab == (MMU_Tab*)0)
	{
		return;
	}
	if(mmu_tab->tab_index == 0 || mmu_tab->tab_index == (MMU_TABLE_NUM - 1))
	{
		return;
	}
	mmu_tab->usage_flag = 0;
}

void Display_Memory_Info(void)
{
#ifdef SYS_MEM_DEBUG
	unsigned char i,j;
	unsigned char mmu_tab_used_num = 0;
	unsigned char mmu_tab_used_index[MMU_TABLE_NUM];

	unsigned char temp8;
	unsigned int temp32;

	static unsigned int g_line = 1;

	for(i = 0 ; i < MMU_TABLE_NUM ; i ++)
	{
		if(DJI_MMU_Tab[i].usage_flag == 1)
		{
			mmu_tab_used_index[mmu_tab_used_num ++] = DJI_MMU_Tab[i].tab_index;
		}
	}

	for(i = 0 ; i < (mmu_tab_used_num - 1) ; i ++)
	{
		for(j = 0; j < (mmu_tab_used_num - i - 1) ; j ++)
		{
			if(DJI_MMU_Tab[mmu_tab_used_index[j]].pmem >
				DJI_MMU_Tab[mmu_tab_used_index[j + 1]].pmem)
			{
				temp8 = mmu_tab_used_index[j + 1];
				mmu_tab_used_index[j + 1] = mmu_tab_used_index[j];
				mmu_tab_used_index[j] = temp8;
			}
		}
	}

	printf("***************(%d)******************\n",g_line++);

	for(i = 0 ; i < mmu_tab_used_num - 1 ; i ++)
	{
		printf("<S=%08X L=%d I=%d,E=%08X>\n",(unsigned long)DJI_MMU_Tab[mmu_tab_used_index[i]].pmem,
				DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size,
				DJI_MMU_Tab[mmu_tab_used_index[i]].tab_index,
				(unsigned long)(DJI_MMU_Tab[mmu_tab_used_index[i]].pmem +
				DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size));

		if(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].pmem > (DJI_MMU_Tab[mmu_tab_used_index[i]].pmem +
				DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size))
		{
			temp32 = (unsigned int)(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].pmem -
					DJI_MMU_Tab[mmu_tab_used_index[i]].pmem) -
					DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size;
			printf("         --idle=%d--\n",temp32);
		}
		else
		{
			printf("         --idle=0--\n");
		}

	}

    printf("<S=%08X L=%d I=%d E=%08X>\n",(unsigned long)DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].pmem,
			DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].mem_size,
			DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].tab_index,
			(unsigned long)(DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].pmem +
			DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].mem_size));
#endif
}

MMU_Tab * Request_Memory(unsigned short size)
{
	unsigned int mem_used = 0;
	unsigned char i;
    unsigned char j = 0;
	unsigned char mmu_tab_used_num = 0;
	unsigned char mmu_tab_used_index[MMU_TABLE_NUM];

	unsigned int temp32;
	unsigned int temp_area[2] = {0xFFFFFFFF,0xFFFFFFFF};

	unsigned int record_temp32 = 0;
	unsigned char magic_flag = 0;

	if(size > PRO_PURE_DATA_MAX_SIZE || size > STATIC_MEMORY_SIZE)
	{
		return (MMU_Tab *)0;
	}

	for(i = 0 ; i < MMU_TABLE_NUM ; i ++)
	{
		if(DJI_MMU_Tab[i].usage_flag == 1)
		{
			mem_used += DJI_MMU_Tab[i].mem_size;
			mmu_tab_used_index[mmu_tab_used_num ++] = DJI_MMU_Tab[i].tab_index;
		}
	}

	if(STATIC_MEMORY_SIZE < (mem_used + size))
	{
		return (MMU_Tab *)0;
	}

	if(mem_used == 0)
	{
		DJI_MMU_Tab[1].pmem = DJI_MMU_Tab[0].pmem;
		DJI_MMU_Tab[1].mem_size = size;
		DJI_MMU_Tab[1].usage_flag = 1;
		return &DJI_MMU_Tab[1];
	}

	for(i = 0 ; i < (mmu_tab_used_num - 1) ; i ++)
	{
		for(j = 0; j < (mmu_tab_used_num - i - 1) ; j ++)
		{
			if(DJI_MMU_Tab[mmu_tab_used_index[j]].pmem >
				DJI_MMU_Tab[mmu_tab_used_index[j + 1]].pmem)
			{
				mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
				mmu_tab_used_index[j] ^= mmu_tab_used_index[j + 1];
				mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
			}
		}
	}

	for(i = 0 ; i < (mmu_tab_used_num - 1) ; i ++)
	{
		temp32 = (unsigned int)(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].pmem -
				 DJI_MMU_Tab[mmu_tab_used_index[i]].pmem);

		if((temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size) >= size)
		{
			if(temp_area[1] > (temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size))
			{
				temp_area[0] = DJI_MMU_Tab[mmu_tab_used_index[i]].tab_index;
				temp_area[1] = temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size;
			}
		}

		record_temp32 += temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size;
		if(record_temp32 >= size && magic_flag == 0)
		{
			j = i;
			magic_flag = 1;
		}
	}

	if(temp_area[0] == 0xFFFFFFFF && temp_area[1] == 0xFFFFFFFF)
	{
		for(i = 0; i < j; i ++)
		{
			 if(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].pmem
					 >  (DJI_MMU_Tab[mmu_tab_used_index[i]].pmem +
					 DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size))
			 {
				 memmove(DJI_MMU_Tab[mmu_tab_used_index[i]].pmem +
						 DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size,
						 DJI_MMU_Tab[mmu_tab_used_index[i + 1]].pmem,
						 DJI_MMU_Tab[mmu_tab_used_index[i + 1]].mem_size);
				 DJI_MMU_Tab[mmu_tab_used_index[i + 1]].pmem = DJI_MMU_Tab[mmu_tab_used_index[i]].pmem +
						 DJI_MMU_Tab[mmu_tab_used_index[i]].mem_size;

				//printf("move tab_index=%d\n",
				//		 DJI_MMU_Tab[mmu_tab_used_index[i + 1]].tab_index);
			 }
		}

		for(i = 1 ; i < (MMU_TABLE_NUM - 1) ; i ++)
		{
			if(DJI_MMU_Tab[i].usage_flag == 0)
			{
				DJI_MMU_Tab[i].pmem =
						DJI_MMU_Tab[mmu_tab_used_index[j]].pmem +
						DJI_MMU_Tab[mmu_tab_used_index[j]].mem_size;

				DJI_MMU_Tab[i].mem_size = size;
				DJI_MMU_Tab[i].usage_flag = 1;
				return &DJI_MMU_Tab[i];
			}
		}
		return (MMU_Tab *)0;
	}

	for(i = 1 ; i < (MMU_TABLE_NUM - 1) ; i ++)
	{
		if(DJI_MMU_Tab[i].usage_flag == 0)
		{
			DJI_MMU_Tab[i].pmem = DJI_MMU_Tab[temp_area[0]].pmem +
									DJI_MMU_Tab[temp_area[0]].mem_size;

			DJI_MMU_Tab[i].mem_size = size;
			DJI_MMU_Tab[i].usage_flag = 1;
			return &DJI_MMU_Tab[i];
		}
	}

	return (MMU_Tab *)0;
}

void Session_Setup(void)
{
	int i;
	for(i = 0; i < SESSION_TABLE_NUM ; i ++)
	{
		DJI_CMD_Session_Tab[i].session_id = i;
		DJI_CMD_Session_Tab[i].usage_flag = 0;
		DJI_CMD_Session_Tab[i].mmu = (MMU_Tab *)NULL;
	}

	for(i = 0; i < (SESSION_TABLE_NUM - 1) ; i ++)
	{
		DJI_ACK_Session_Tab[i].session_id = i + 1;
		DJI_ACK_Session_Tab[i].session_status = ACK_SESSION_IDLE;
		DJI_ACK_Session_Tab[i].mmu = (MMU_Tab *)NULL;
	}
}

/* request a cmd session for sending cmd data
 * when arg session_id = 0/1, which means select session 0/1 to send cmd
 * otherwise set arg session_id = CMD_SESSION_AUTO (32), which means auto select a idle session id between 2~31.
 */

CMD_Session_Tab * Request_CMD_Session(unsigned short session_id,unsigned short size)
{
	int i;
	MMU_Tab *mmu = NULL;

	if(session_id == 0 || session_id == 1)
	{
		if(DJI_CMD_Session_Tab[session_id].usage_flag == 0)
		{
			i = session_id;
		}
		else
		{
			/* session is busy */
			printf("%s:%d:ERROR,session %d is busy\n",__func__,__LINE__,session_id);
			return NULL;
		}
	}
	else
	{
		for(i = 2 ; i < SESSION_TABLE_NUM ; i ++)
		{
			if(DJI_CMD_Session_Tab[i].usage_flag == 0)
			{
				break;
			}
		}
	}
	if(i < 32 && DJI_CMD_Session_Tab[i].usage_flag == 0)
	{
		DJI_CMD_Session_Tab[i].usage_flag = 1;
		mmu = Request_Memory(size);
		if(mmu == NULL)
		{
			DJI_CMD_Session_Tab[i].usage_flag = 0;
		}
		else
		{
			DJI_CMD_Session_Tab[i].mmu = mmu;
			return &DJI_CMD_Session_Tab[i];
		}
	}
	return NULL;
}

void Free_CMD_Session(CMD_Session_Tab * session)
{
	if(session->usage_flag == 1)
	{
		Free_Memory(session->mmu);
		session->usage_flag = 0;
	}
}

ACK_Session_Tab * Request_ACK_Session(unsigned short session_id,unsigned short size)
{
	MMU_Tab *mmu = NULL;
	if(session_id > 0 && session_id < 32)
	{
		if(DJI_ACK_Session_Tab[session_id - 1].mmu)
		{
			Free_Memory(DJI_ACK_Session_Tab[session_id - 1].mmu);
		}
		mmu = Request_Memory(size);
		if(mmu == NULL)
		{
		}
		else
		{
			DJI_ACK_Session_Tab[session_id - 1].mmu = mmu;
			return &DJI_ACK_Session_Tab[session_id - 1];
		}
	}
	return NULL;
}

void Free_ACK_Session(ACK_Session_Tab * session)
{
	Free_Memory(session->mmu);
}

void DJI_Pro_Rmu_Setup(void)
{
	MMU_Setup();
	Session_Setup();
}
