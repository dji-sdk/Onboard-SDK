//
//  downloadbufferqueue.cpp
//  djisdk
//
//  Created by husong on 12/27/17.
//
//

#include "downloadbufferqueue.h"
#include <assert.h>
#include <stdio.h>
#include <chrono>
#include <cstring>

#ifdef ANDROID
#include <malloc.h>
#endif

using namespace std::chrono;

namespace DJI {
namespace OSDK {
bool DownloadBufferQueue::InitBufferQueue(int size, int start_index) {
    if (m_queue_ptr != nullptr) {
        free(m_queue_ptr);
        m_queue_ptr = nullptr;
    }
    m_queue_ptr = (DataPointer *)malloc(sizeof(DataPointer) * size);

    memset(m_queue_ptr, 0x00, sizeof(DataPointer) * size);

    m_size = size;
    m_expect_index = start_index;
    m_head = m_expect_index % m_size;
    m_buf_max_index = m_expect_index - 1;

    return true;
}

// flag 代表是否覆盖已有队列缓存
DownloadBufferQueue::InsertRetType DownloadBufferQueue::InsertBlock(const uint8_t *pack, uint32_t data_length, int index, bool flag) {
  std::lock_guard<std::mutex> lock(m_mutex);
  InsertRetType ret = INSERT_FAIL_UNKOWN;

  if (data_length <= 0) {
    return INSERT_FAIL_INVALID_PARAM;
  }

  if (m_expect_index + m_size > index && m_expect_index <= index) {
    int insert_index = m_head + (index - m_expect_index);
    insert_index = insert_index % m_size;
    DataPointer data_ptr = m_queue_ptr[insert_index];

    if (data_ptr.data) {
      if (false == flag) {
        return INSERT_FAIL_MEMORY_USED;
      }

      free(data_ptr.data);
    }

    data_ptr.data = (uint8_t *) malloc(data_length);
    data_ptr.length = data_length;
    memcpy(data_ptr.data, pack, data_length);

    m_queue_ptr[insert_index] = data_ptr;

    if (index > m_buf_max_index) {
      m_buf_max_index = index;
    }

    ret = INSERT_SUCCESS;
   // printf("log .................................index = %d; m_expect_index + m_size = %d\n", index, m_expect_index + m_size);
    if (index == (m_expect_index + m_size - 1)) {
     // printf("满了 .................................index = %d; m_expect_index + m_size = %d\n", index, m_expect_index + m_size);
      return INSERT_SUCCESS_FULL;
    }
  } else {
    ret = INSERT_FAIL_OUT_OF_RANGE;
  }

  return ret;
}

bool DownloadBufferQueue::FindBlockByIndex(int index) {
    bool ret = false;

    if (m_expect_index + m_size > index && index >= m_expect_index) {
        int queue_index = index - m_expect_index + m_head;
        queue_index = queue_index % m_size;
        DataPointer data_ptr = m_queue_ptr[queue_index];

        if (data_ptr.data) {
            ret = true;
        }
    }

    return ret;
}

int DownloadBufferQueue::GetConfirmSeq() {
    return m_expect_index - 1;
}

int DownloadBufferQueue::GetBufMaxSeq() {
    return m_buf_max_index;
}

DataPointer DownloadBufferQueue::DequeueBuffer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    DataPointer data_ptr = m_queue_ptr[m_head % m_size];

    if (!data_ptr.data || data_ptr.length == 0) {
        DataPointer nil_ptr = {nullptr};
        return nil_ptr;
    }

    m_queue_ptr[m_head % m_size].data = nullptr;
    m_queue_ptr[m_head % m_size].length = 0;

    m_expect_index++;
    m_head++;
    m_head = m_head % m_size;

    return data_ptr;
}

std::list<DataPointer> DownloadBufferQueue::DequeueAllBuffer() {
    DataPointer data_pointer = {nullptr};
    std::list<DataPointer> data_pointer_list;
    data_pointer_list.clear();

    while ((data_pointer = DequeueBuffer()).data != nullptr) {
        data_pointer_list.push_back(data_pointer);
    }
    return data_pointer_list;
}

void DownloadBufferQueue::Clear() {
    for (int i = 0; i < m_size; i++) {
        if (m_queue_ptr[i].data) {
            free(m_queue_ptr[i].data);
            m_queue_ptr[i].data = nullptr;
            m_queue_ptr[i].length = 0;
        }
    }
}

void DownloadBufferQueue::Dealloc() {
    Clear();

    if (m_queue_ptr) {
        free(m_queue_ptr);
        m_queue_ptr = nullptr;
    }

    m_size = 0;
}
}  // namespace OSDK

}  // namespace DJI
