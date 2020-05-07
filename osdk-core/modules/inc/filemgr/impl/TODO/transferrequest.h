//
//  newcommondatarequest.h
//  djisdk
//
//  Created by Xietong LU on 2018/5/27.
//

#ifndef newcommondatarequest_h
#define newcommondatarequest_h

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <cstring>
#include <functional>
#include "commondatarangehandler.h"
#include "downloadbufferqueue.h"
#include "filetransferprotocoldef.h"
#include "dji_file_mgr_internal_define.hpp"
#include "osdk_protocol_common.h"
#include "osdk_typedef.h"
#include "csdkErrorCode.hpp"

namespace DJI {
namespace OSDK {
struct RequestCommandData {
    dji_general_transfer_msg_req header;
    size_t data_size = 0;
    uint8_t *data = nullptr;
    uint8_t receiver_type = OSDK_COMMAND_DEVICE_TYPE_CAMERA;
    uint8_t receiver_index = 0;

    int ToBytes(dji_general_transfer_msg_req *bytes) const {
        size_t header_size = sizeof(dji_general_transfer_msg_req) - 1;
        memcpy(&bytes[0], &header, sizeof(header));
        if (data_size > 0 && data != nullptr)
            memcpy(((uint8_t *)bytes) + header_size, data, data_size);
        return (int)(header_size + data_size);
    }

    int GetFullSize() const {
        size_t header_size = sizeof(dji_general_transfer_msg_req) - 1;
        return (int)(header_size + data_size);
    }

    void ResizeData(size_t data_size_t) {
        // Assume RequestCommandData can only be resized once.
        assert(data_size == 0 && data == nullptr);
        if (data_size_t == data_size) {
            return;
        }
        if (data) {
            free(data);
        }
        data_size = data_size_t;
        data = (uint8_t *)malloc(data_size);
        memset(data, 0, data_size);
        size_t header_size = sizeof(dji_general_transfer_msg_req) - 1;
        header.msg_length = data_size + header_size;
    }

    RequestCommandData(uint32_t session = 0, uint32_t seq = 0, DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE task_id = DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST,
                       DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE func_id = DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_REQ) {
        header.func_id = func_id;
        header.task_id = task_id;
        header.version = 0x01;
        header.header_length = sizeof(header) - 1;  // Pay attention here.
        header.msg_flag = 1;
        header.msg_length = 0;
        header.seq = seq;
        header.session_id = session;
    }

    ~RequestCommandData() {
        if (data != nullptr) {
            free(data);
        }
    }
};

/*! 0x0026 download ack */
typedef struct TransmissionMissedSection
{
  uint32_t from;
  uint32_t count;
} TransmissionMissedSection;

/*! 0x0026 download ack */
typedef struct TransmissionMissedSections
{
  uint32_t ackSeq;
  uint8_t count;
  std::vector<TransmissionMissedSection> sections;
} TransmissionMissedSections;


class TransferRequest {
public:
    enum class DataParseResult {
        CONTINUE,  // expecting the next seq.
        DONE,
        FAILED,
    };
    TransferRequest(uint16_t session_id, int buffer_size = 100);
    virtual ~TransferRequest();

    uint16_t GetSessionID() {
        return session_id_;
    }

    virtual int GetConfirmedSeq();
    virtual bool ShouldResendAll();
    virtual std::vector<Range> &GetNotAckedRanges();

    //virtual dji_general_transfer_msg_req CreateStartRequestPack() = 0;
    //dji_general_transfer_msg_req CreatePack(const RequestCommandData &command_data);
    //dji_general_transfer_msg_req CreateAbortPack();
    //dji_general_transfer_msg_req CreateACKPack(std::shared_ptr<const TransmissionMissedSections> range);

    using ParsingResultHandler = std::function<void(TransferRequest::DataParseResult, E_OsdkStat err_code)>;
    virtual void OnReceiveDataPack(dji_general_transfer_msg_ack *rsp, ParsingResultHandler handler);

    virtual void OnHandlerFailure(int err_code) {}
    // Called by the handler when the error is initiated by the handler
    // Otherwise, the request itself is responsible for reseting itself.
    virtual void ResetInternalData() {}
    virtual std::string GetDescription() {
        return "A common data request with session id: " + std::to_string(session_id_);
    }

    // Timeout for the session in millisecond.
    virtual int GetTimeoutDuration() {
        return 3000;
    }

    virtual int GetAbortSessionTaskID() { 
        return DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST;
    }

protected:
    virtual bool ShouldParseData() {
        return true;
    }
    virtual void ParseData(bool is_last_pack, ParsingResultHandler handler) = 0;

    uint16_t session_id_ = 0;
    //uint8_t receiver_type_ = OSDK_COMMAND_DEVICE_TYPE_PC;
    //uint8_t receiver_index_ = 6;

    std::shared_ptr<CommonDataRangeHandler> range_handler_;
    std::shared_ptr<DownloadBufferQueue> download_buffer_;
    bool is_last_pack_buffered_ = false;
    int last_pack_seq_ = 0;
    bool IsLastPack();
    // Send Recent ack while seq received exceeds buffer size
    bool should_recent_all_ = false;
};
}  // namespace OSDK
}  // namespace DJI

#endif /* newcommondatarequest_h */
