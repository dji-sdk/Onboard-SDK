//
//  newfiledatarequest.cpp
//  djisdk
//
//  Created by Xietong LU on 2018/6/10.
//

#include "impl/TODO/datatransferrequest.h"

using namespace DJI;
using namespace DJI::OSDK;

#define PARSE_INTERVAL (0.1)
static const std::chrono::milliseconds parse_interval((int)(PARSE_INTERVAL * 1000));

DataTransferRequest::DataTransferRequest(uint16_t session_id, std::shared_ptr<const FileDataRequest> config, FileDataCallback callback)
    : TransferRequest(session_id, 5000), config_(config), file_data_callback_(callback), rate_calculator_(100) {
    if (config != nullptr)
        type_ = DJI_DOWNLOAD_FILE_SUBTYPE(config->type);
}

DataTransferRequest::~DataTransferRequest() {}

dji::core::dji_cmd_req DataTransferRequest::CreateStartRequestPack() {
    assert(ValidateRequestConfig() == kNoError);

    RequestCommandData command_data(GetSessionID());
    command_data.ResizeData(sizeof(DownloadRequest));
    DownloadRequest *request = (DownloadRequest *)command_data.data;
    request->file_count = config_->count;
    request->startIndex = config_->index;
    request->sub_type = (uint8_t)config_->type;
    request->offset = (uint32_t)config_->offSet;
    request->data_size = config_->dataSize;
    request->index = config_->subIndex;
    request->seg_sub_index = config_->segSubIndex;
    if (request->data_size == 0) {
        request->data_size = -1;
    }

    /*
    LOGD << "[DEBUG - PLAYBACK] Common File Data - file_count:" << request->file_count
    << ", startIndex:" << request->startIndex
    << ", sub_type:" << (int)request->sub_type
    << ", offset:" << request->offset
    << ", data_size:" << request->data_size
    << ", file_index: " << (int)request->index;
    */

    command_data.header.task_id = DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE;
    return CreatePack(command_data);
}

int DataTransferRequest::ValidateRequestConfig() {
    if (config_ == nullptr) {
        return kErrorInvalidParam;
    }
    return kNoError;
}

void DataTransferRequest::OnHandlerFailure(int err_code) {
    LOGE << "[FileTransferRequest] " << GetDescription() << " OnHandlerFailure retCode " << err_code;
    TriggerCallback(err_code, false, 0.0, Dji::Common::Buffer());
}

void DataTransferRequest::ResetInternalData() {}

bool DataTransferRequest::ShouldParseData() {
    return (std::chrono::system_clock::now() - last_parsing_time_ > parse_interval);
}

void DataTransferRequest::ParseData(bool is_last_pack, ParsingResultHandler handler) {
    last_parsing_time_ = std::chrono::system_clock::now();
    //下载header 和moov 需要整个包上传
    if (patch_data_ && !is_last_pack) {
        if (handler)
            handler(DataParseResult::CONTINUE, kNoError);
        return;
    }

    std::list<DataPointer> data_list;
    if (patch_data_ && is_last_pack) {
        data_list = download_buffer_->DequeueAllBuffer();

        if (data_list.size() <= 0) {
            //            LOGE << "It is the last pack but the buffer is empty. ";
            if (handler)
                handler(DataParseResult::FAILED, kErrorSystemError);
            return;
        }
    } else {
        data_list = download_buffer_->DequeueAllBuffer();
    }

    if (data_list.size() <= 0) {
        //        LOGD << "Get nothing from the buffer. ";
        if (handler)
            handler(DataParseResult::CONTINUE, kNoError);
        return;
    }

    Dji::Common::Buffer send_buffer;

    while (!data_list.empty()) {
        DataPointer data_pointer = data_list.front();
        data_list.pop_front();

        if (data_pointer.data == nullptr || data_pointer.length == 0) {
            continue;
        }

        dji_general_transfer_msg_ack *msg_data = (dji_general_transfer_msg_ack *)data_pointer.data;

        uint8_t *buffer = msg_data->data;
        auto data_size = msg_data->msg_length - (sizeof(dji_general_transfer_msg_ack) - 1);

        // LOGD<<"GetDownloadBuffer:"<< msg_data->seq <<", msg length:" << msg_data->msg_length;

        if (msg_data->seq == 0) {
            FileFragment *fragPtr = (FileFragment *)(((FileData *)(msg_data->data))->frag);

            data_size = data_size - (sizeof(FileData) - 1) - fragPtr->path_len;
            //            request_data_size_ = fragPtr->size - (sizeof(FileData) - 1);
            buffer = fragPtr->data;
        }

        send_buffer.append(buffer, (int)data_size);
        //        delivery_data_size_ += data_size;

        free(data_pointer.data);
    }

    rate_calculator_.UpdateByteLength(send_buffer.size());
    auto last_rate_ = rate_calculator_.GetRate();
    if (is_last_pack && send_buffer.size() == 0 && patch_data_) {
        if (handler)
            handler(DataParseResult::FAILED, kErrorDownloadedEmptyData);
        TriggerCallback(kErrorDownloadedEmptyData, true, 0.0, send_buffer);
        return;
    } else if (is_last_pack) {
        if (handler)
            handler(DataParseResult::DONE, kNoError);
        TriggerCallback(kNoError, true, last_rate_ * 8, send_buffer);
        return;
    } else {
        TriggerCallback(kNoError, false, last_rate_ * 8, send_buffer);
        if (handler)
            handler(DataParseResult::CONTINUE, kNoError);
        return;
    }
}

void DataTransferRequest::TriggerCallback(int err_code, bool is_end, double bit_per_second, const Dji::Common::Buffer &data_buf) {
    if (file_data_callback_ == nullptr) {
        return;
    }
    auto copy = file_data_callback_;
    if (is_end || err_code != kNoError) {
        file_data_callback_ = nullptr;
    }
    if (is_end || err_code != 0) {
        //        LOGD << "Last call for session: " << GetSessionID();
    }
    copy(err_code, is_end, bit_per_second, data_buf);
}

std::string DataTransferRequest::GetDescription() {
    return ("Data request session: " + std::to_string(GetSessionID()) + " index: " + std::to_string(config_->index) +
            " subIndex: " + std::to_string(config_->subIndex) + " type: " + std::to_string((int)(config_->type)));
}

int DataTransferRequest::GetAbortSessionTaskID() {
    return DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE;
}
