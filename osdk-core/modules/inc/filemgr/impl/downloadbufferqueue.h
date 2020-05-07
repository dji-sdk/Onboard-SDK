//
//  downloadbufferqueue.h
//  djisdk
//
//  Created by husong on 12/27/17.
//
//

#ifndef downloadbufferqueue_h
#define downloadbufferqueue_h

#include <stdio.h>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>

namespace DJI {
namespace OSDK {
typedef struct DJIDataPointer {
    void* data;
    int length;
} DataPointer;

class DownloadBufferQueue final {
public:
  typedef enum InsertRetType {
    INSERT_SUCCESS = 0,
    INSERT_SUCCESS_FULL,
    INSERT_FAIL_INVALID_PARAM,
    INSERT_FAIL_MEMORY_USED,
    INSERT_FAIL_OUT_OF_RANGE,
    INSERT_FAIL_UNKOWN,
  } InsertRetType;

    DownloadBufferQueue() = default;
    virtual ~DownloadBufferQueue() = default;

    DownloadBufferQueue(const DownloadBufferQueue& other) = delete;
    DownloadBufferQueue(DownloadBufferQueue&& other) = delete;
    DownloadBufferQueue& operator=(const DownloadBufferQueue& other) = delete;
    DownloadBufferQueue& operator=(DownloadBufferQueue&& other) = delete;

    bool InitBufferQueue(int size, int start_index);

    bool FindBlockByIndex(int index);
    InsertRetType InsertBlock(const uint8_t *data, uint32_t data_length, int index, bool flag);

    DataPointer DequeueBuffer();
    std::list<DataPointer> DequeueAllBuffer();
    int GetConfirmSeq();
    int GetBufMaxSeq();
    int GetSize() {
        return m_size;
    };

    void Clear();
    void Dealloc();

private:
    mutable std::mutex m_mutex;
    std::condition_variable m_data_condition;
    DataPointer* m_queue_ptr = nullptr;

    // 确认收到并缓存最大index
    int m_buf_max_index;
    // 期待接受的index， 即确认收到连续序列index + 1
    int m_expect_index;
    // 期待接受的index对应的Buf数组下标
    int m_head;
    // Buffer 的大小
    int m_size;
};
}  // namespace OSDK

}  // namespace DJI

#endif /* downloadbufferqueue_h */
