//
//  newcommondatarequest.cpp
//  djisdk
//
//  Created by Xietong LU on 2018/5/27.
//

#include "impl/TODO/transferrequest.h"

using namespace DJI;
using namespace DJI::OSDK;

TransferRequest::TransferRequest(uint16_t session_id, int buffer_size) : session_id_(session_id) {
    download_buffer_ = std::make_shared<DownloadBufferQueue>();
    download_buffer_->InitBufferQueue(buffer_size, 0);
    range_handler_ = std::make_shared<CommonDataRangeHandler>();
}

TransferRequest::~TransferRequest() {
    if (download_buffer_) {
        download_buffer_->Dealloc();
        download_buffer_ = nullptr;
    }
}

bool TransferRequest::IsLastPack() {
    if (range_handler_ == nullptr) {
        return false;
    }

    return is_last_pack_buffered_ && (range_handler_->GetLastNotReceiveSeq() == last_pack_seq_ + 1) && (range_handler_->GetNoAckRanges().size() == 0);
}

void TransferRequest::OnReceiveDataPack(dji_general_transfer_msg_ack *rsp, ParsingResultHandler handler) {
    if (download_buffer_ == nullptr) {
        if (handler)
            handler(DataParseResult::FAILED, OSDK_STAT_ERR_ALLOC);
        return;
    }

    bool buffer_result = download_buffer_->InsertBlock((const uint8_t *)rsp, rsp->msg_length, rsp->seq, true);

    should_recent_all_ = !buffer_result;
    if (!buffer_result) {
        return;
    }

    if (range_handler_ == nullptr) {
        if (handler)
            handler(DataParseResult::FAILED, OSDK_STAT_ERR_ALLOC);
        return;
    }
    range_handler_->AddSeqIndex(rsp->seq, download_buffer_->GetConfirmSeq(), download_buffer_->GetSize());

    if (rsp->msg_flag & 0x01) {
        is_last_pack_buffered_ = true;
        last_pack_seq_ = rsp->seq;
    }
    bool is_last_pack = IsLastPack();
    if (ShouldParseData() || is_last_pack) {
        ParseData(is_last_pack, handler);
        return;
    } else {
        if (handler)
            handler(DataParseResult::CONTINUE, OSDK_STAT_OK);
        return;
    }
}

int TransferRequest::GetConfirmedSeq() {
    if (download_buffer_) {
        return download_buffer_->GetConfirmSeq();
    }
    return -1;
}

bool TransferRequest::ShouldResendAll() {
    return should_recent_all_;
}

std::vector<Range> &TransferRequest::GetNotAckedRanges() {
    if (range_handler_) {
        return range_handler_->GetNoAckRanges();
    }
    static std::vector<Range> dummy_range;
    return dummy_range;
}

/*
dji_general_transfer_msg_req TransferRequest::CreatePack(const RequestCommandData &command_data) {
    int full_size = command_data.GetFullSize();
    dji_general_transfer_msg_req *pack_data = (dji_general_transfer_msg_req *)malloc(full_size);
    command_data.ToBytes(pack_data);

    core::file_transfer_req pack;
    pack.set_req_body(*pack_data, full_size);

    free(pack_data);

    pack.receiver_type = command_data.receiver_type;
    pack.receiver_index = command_data.receiver_index;
    pack.retry_times = 5;
    pack.retry_interval = 1000;

    // LOGI << "SendData with session:" << command_data.header.session_id << ", type:" << (int)command_data.header.func_id << ", type: " << (int)command_data.header.task_id;
    return std::move(pack);
}

dji_general_transfer_msg_req TransferRequest::CreateAbortPack() {
    RequestCommandData command_data(GetSessionID());
    command_data.ResizeData(sizeof(uint32_t));
    uint32_t *cancel_reason = (uint32_t *)command_data.data;
    *cancel_reason = (uint32_t)TransAbortReason::TransAbortReasonForce;
    command_data.header.func_id = DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ABORT;
    command_data.header.task_id = GetAbortSessionTaskID();
    // set receiver_type here
    // if (type) == ??
    //    command_data.receiver_type = ??;
    return CreatePack(command_data);
}

dji_general_transfer_msg_req TransferRequest::CreateACKPack(std::shared_ptr<const TransmissionMissedSections> range) {
    assert(range != nullptr);

    RequestCommandData command_data(GetSessionID());

    int seg_count = range->count;
    size_t head_size = sizeof(CommonFileAck);
    size_t seg_size = sizeof(TransMissSegmentRange) * seg_count;
    size_t total_size = head_size + seg_size;

    command_data.ResizeData(total_size);
    command_data.header.func_id = DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ACK;

    CommonFileAck *request = (CommonFileAck *)command_data.data;
    request->ackSeq = range->ackSeq;
    request->missSegmentCount = range->count;

    TransMissSegmentRange *segments = (TransMissSegmentRange *)(((uint8_t *)request) + head_size);
    for (int i = 0; i < seg_count; i++) {
        segments[i].startSeqNum = range->sections[i].from;
        segments[i].fragCount = range->sections[i].count;
    }

    return CreatePack(command_data);
}
*/