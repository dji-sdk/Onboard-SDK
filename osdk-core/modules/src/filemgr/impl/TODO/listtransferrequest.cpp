//
//  newfilelistrequest.cpp
//  djisdk
//
//  Created by Xietong LU on 2018/5/27.
//

#include "impl/TODO/listtransferrequest.h"

using namespace DJI;
using namespace DJI::OSDK;

ListTransferRequest::ListTransferRequest(uint16_t session_id, std::shared_ptr<const FileListRequest> config, DJI_CAMERA_TYPE camera_type,
                                         FileListCallback callback)
    : TransferRequest(session_id), config_(config), camera_type_(camera_type) {
    output_handler_ = std::make_shared<FileListOutputHandler>(callback);
    if (config != nullptr) {
        if (config->type == FileType::COMMON) {
            type_ = DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_LOG;
        } else {
            if (config->isAllList) {
                type_ = DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_MEDIA;
            } else {
                type_ = DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_MEDIA_GROUP;
            }
        }
    };
}

#if 0
ListTransferRequest::ListTransferRequest(uint16_t session_id, std::shared_ptr<const FileListRequest> config, SpeakerAudioFileListCallback callback)
    : TransferRequest(session_id), config_(config) {
    output_handler_ = std::make_shared<SpeakerFileListOutputHandler>(callback);
    type_ = DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_MEDIA;
    receiver_type_ = DJI_SENDTYPE_AIROFDM;
    receiver_index_ = 3;
}
#endif

dji::sdk::ListTransferRequest::~ListTransferRequest() {
    for (auto &data : data_buffers_) {
        free(data.first);
    }
    data_buffers_.clear();
}

void ListTransferRequest::ConfigFilterData() {
    for (auto const &value : config_->filterlist) {
        switch (FileListRequestFilter(value)) {
            case FileListRequestFilter::ALL:
                photoFilter_ = -1;
                videoFilter_ = -1;
                break;
            case FileListRequestFilter::LIKED:
                likeFilter_ = DJI_FILE_LIST_FILTER_LIKE::DJI_like;
                break;
            case FileListRequestFilter::DISLIKED:
                likeFilter_ = DJI_FILE_LIST_FILTER_LIKE::DJI_dislike;
                break;
            case FileListRequestFilter::PHOTO_NORMAL:
                photoFilter_ |= 1 << 0;
                break;
            case FileListRequestFilter::PHOTO_HDR:
                photoFilter_ |= 1 << 1;
                break;
            case FileListRequestFilter::PHOTO_AEB:
                photoFilter_ |= 1 << 2;
                break;
            case FileListRequestFilter::PHOTO_INTERVAL:
                photoFilter_ |= 1 << 3;
                break;
            case FileListRequestFilter::PHOTO_BURST:
                photoFilter_ |= 1 << 4;
                break;
            case FileListRequestFilter::PHOTO_PANO:
                photoFilter_ |= 1 << 5;
                break;
            case FileListRequestFilter::VIDEO_NORMAL:
                videoFilter_ |= 1 << 0;
                break;
            case FileListRequestFilter::VIDEO_SLOWMOTION:
                videoFilter_ |= 1 << 1;
                break;
            case FileListRequestFilter::VIDEO_TIMELAPSE:
                videoFilter_ |= 1 << 2;
                break;
            case FileListRequestFilter::VIDEO_HYPERLAPSE:
                videoFilter_ |= 1 << 3;
                break;
            case FileListRequestFilter::VIDEO_HDR:
                videoFilter_ |= 1 << 4;
                break;
            case FileListRequestFilter::VIDEO_LOOP:
                videoFilter_ |= 1 << 5;
                break;
            default:
                break;
        }
    }
}

dji::core::dji_cmd_req ListTransferRequest::CreateStartRequestPack() {
    assert(ValidateRequestConfig() == kNoError);

    bool isFilterEnable = !(config_->filterlist.empty());
    if (isFilterEnable) {
        ConfigFilterData();
    }

    RequestCommandData command_data(GetSessionID());

    if (config_->type == FileType::SPEAKER_AUDIO) {
        command_data.ResizeData(sizeof(AudioFileListRequest));
        AudioFileListRequest *request = (AudioFileListRequest *)command_data.data;
        request->file_index.index = 1;
        request->file_index.drive = 0;
        request->file_count = config_->isAllList ? -1 : config_->count;
        request->sub_type = (uint8_t)type_;
        command_data.receiver_type = receiver_type_;
        command_data.receiver_index = receiver_index_;
    } else {
        command_data.ResizeData(sizeof(ListRequest));
        ListRequest *request = (ListRequest *)command_data.data;
        request->startIndex = uint32_t(config_->index);
        request->file_count = uint16_t(config_->count);
        request->sub_type = uint8_t(type_);
        if (isFilterEnable) {
            ListRequestFilter filters;
            filters.like = likeFilter_;
            filters.video = videoFilter_;
            filters.photo = photoFilter_;
            request->filter_enable = 1;
            request->filters = filters;
        } else {
            request->filter_enable = 0;
        }
    }

    return CreatePack(command_data);
}

int ListTransferRequest::ValidateRequestConfig() {
    if (config_ == nullptr) {
        return kErrorInvalidParam;
    }
    return kNoError;
}

void ListTransferRequest::OnHandlerFailure(int err_code) {
    output_handler_->TriggerCallback(err_code);
}

size_t ListTransferRequest::TotalBufferSize() {
    size_t total_size = 0;
    for (auto &data : data_buffers_) {
        total_size += data.second;
    }
    return total_size;
}

void ListTransferRequest::ParseData(bool is_tail, ParsingResultHandler handler) {
    auto data_list = download_buffer_->DequeueAllBuffer();
    if (data_list.size() <= 0) {
        if (handler)
            handler(DataParseResult::CONTINUE, kNoError);
        return;
    }

    for (auto itr = data_list.begin(); itr != data_list.end(); itr++) {
        auto pack = (dji_general_transfer_msg_ack *)itr->data;
        size_t pack_data_size = pack->msg_length - sizeof(dji_general_transfer_msg_ack) + 1;
        // FIX ME: 有点别扭。因为不想改原来的逻辑，所以用这种方法，冲用了旧逻辑。
        StoreData((uint8_t *)pack->data, pack_data_size, false);
    }

    size_t cur_size = TotalBufferSize();

    uint8_t *chunk_buf = nullptr;
    size_t chunk_size = 0;  // Pack 大小
    size_t chunk_ind = 0;   // 已解析 Pack 大小

    // Todo: make it static ???
    std::function<void()> TryGetNextBuffer = [this, &chunk_size, &chunk_buf, &chunk_ind]() {
        if (!(chunk_buf == nullptr && chunk_size == 0 && chunk_ind == 0)) {
            return;
        }
        auto &buffer = data_buffers_.front();
        chunk_buf = buffer.first;
        chunk_size = buffer.second;
        data_buffers_.pop_front();
    };

    // Todo: make it static ???
    std::function<void(size_t, size_t &, uint8_t *)> ConsumeBuffer = [&chunk_size, &chunk_buf, &chunk_ind, &cur_size](size_t item_size, size_t &item_index,
                                                                                                                      uint8_t *item_buf) {
        if (chunk_size == 0 && chunk_buf == nullptr) {
            return;
        }
        size_t size_to_copy = std::min(item_size - item_index, chunk_size);
        memcpy(item_buf + item_index, chunk_buf + chunk_ind, size_to_copy);
        item_index += size_to_copy;
        chunk_ind += size_to_copy;
        cur_size -= size_to_copy;
        if (chunk_size == chunk_ind) {
            // Pack 已解析完，释放内存
            free(chunk_buf);
            chunk_buf = nullptr;
            chunk_ind = 0;
            chunk_size = 0;
        }
    };

    if (parse_state_ == ParseState::INITIAL) {
        static const size_t header_size = sizeof(ListDataPack) - sizeof(ListDataItem);
        if (cur_size < header_size) {
            if (handler)
                handler(DataParseResult::CONTINUE, kNoError);
            return;
        }

        assert(header_ == nullptr);
        header_ = (ListDataPack *)malloc(sizeof(ListDataPack));
        memset(header_, 0, sizeof(ListDataPack));
        size_t header_ind = 0;
        while (true) {
            TryGetNextBuffer();
            ConsumeBuffer(header_size, header_ind, (uint8_t *)header_);
            if (header_ind == header_size) {
                break;
            }
        }
        LOGI << "[FileMgr] ParseData ParseState::INITIAL ReceiceFileCount: " << header_->file_count;
        parse_state_ = ParseState::PARSING_ITEM_HEADER;
    }

    while (cur_size > 0) {
        if (parse_state_ == ParseState::PARSING_ITEM_HEADER) {
            static const size_t item_header_size = sizeof(ListDataItem) - sizeof(uint8_t);
            if (cur_size < item_header_size) {
                break;
            }

            assert(item_header_ == nullptr);
            item_header_ = (ListDataItem *)malloc(item_header_size);
            memset(item_header_, 0, item_header_size);
            size_t item_header_ind = 0;
            while (true) {
                TryGetNextBuffer();
                ConsumeBuffer(item_header_size, item_header_ind, (uint8_t *)item_header_);
                if (item_header_ind == item_header_size) {
                    break;
                }
            }
            parse_state_ = ParseState::PARSING_ITEM_BODY;
        }

        if (parse_state_ == ParseState::PARSING_ITEM_BODY) {
            assert(item_header_ != nullptr);
            size_t body_size = item_header_->path_length;
            if (cur_size < body_size && !is_tail) {
                break;
            }

            // Handle the case that firmware don't send complete data
            if (is_tail && cur_size < body_size) {
                LOGE << "[FileMgr] No more data but the remain data is shorten than expected. ";
                item_header_->path_length = cur_size;
                body_size = cur_size;
            }

            size_t body_ind = 0;
            uint8_t *body_buf = (uint8_t *)malloc(body_size);
            while (true && body_size > 0) {
                TryGetNextBuffer();
                ConsumeBuffer(body_size, body_ind, (uint8_t *)body_buf);
                if (body_ind == body_size) {
                    break;
                }
            }
            switch (type_) {
                case DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_MEDIA: {
                    if (config_->type == FileType::SPEAKER_AUDIO) {
                        ConsumeSpeakerAudioItem(item_header_, body_buf);
                    } else {
                        ConsumeMediaItem(item_header_, body_buf);
                    }
                    break;
                }
                case DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_MEDIA_GROUP:
                    ConsumeMediaItem(item_header_, body_buf);
                    break;
                case DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_LOG:
                    ConsumeCommonItem(item_header_, body_buf);
                    break;
                default:
                    break;
            }
            free(body_buf);
            body_buf = nullptr;
            free(item_header_);
            item_header_ = nullptr;
            // It is possible to invalid file list item, but we will continue the parsing
            //            if (ret_code != kNoError) {
            //                parse_state_ = ParseState::FAILED;
            //                break;
            //            }
            parse_state_ = ParseState::PARSING_ITEM_HEADER;
        }
    }

    // Push the remaining data back to buffers.
    if (chunk_ind < chunk_size) {
        StoreData(chunk_buf + chunk_ind, chunk_size - chunk_ind, true);
        free(chunk_buf);
        chunk_ind = 0;
        chunk_size = 0;
    }

    if (parse_state_ == ParseState::FAILED) {
        if (handler)
            handler(DataParseResult::FAILED, kErrorDownloadDataParsingFailure);
        return;
    }

    if (is_tail) {
        output_handler_->OnTail(header_->file_count);

        if (header_) {
            free(header_);
            header_ = nullptr;
        }

        parse_state_ = ParseState::DONE;
        if (handler)
            handler(DataParseResult::DONE, kNoError);
        output_handler_->TriggerCallback(kNoError);
        return;
    }
    if (handler)
        handler(DataParseResult::CONTINUE, kNoError);
    return;
}

void ListTransferRequest::StoreData(uint8_t *data, size_t data_size, bool front) {
    uint8_t *data_cpy = (uint8_t *)malloc(data_size);
    memcpy(data_cpy, data, data_size);
    if (front) {
        data_buffers_.push_front({data_cpy, data_size});
    } else {
        data_buffers_.push_back({data_cpy, data_size});
    }
}

int ListTransferRequest::ConsumeCommonItem(ListDataItem *item, uint8_t *data) {
    if (item == nullptr || data == nullptr) {
        return kErrorSystemError;
    }
    std::shared_ptr<FileListOutputHandler> handler = std::static_pointer_cast<FileListOutputHandler>(output_handler_);
    std::shared_ptr<FileList> &output = handler->output_;
    if (output == nullptr) {
        output = std::make_shared<FileList>();
        output->hasInvalidFile = false;
        output->files.type = FileType::COMMON;
    }
    std::shared_ptr<CommonFile> file_item = RemoteFileHelper::GetCommonFileHelper(*item, data);

    if (file_item && (uint8_t)file_item->fileType != 0xff) {
        output->files.common.push_back(*file_item.get());
        return kNoError;
    }

    output->hasInvalidFile = true;
    LOGE << "[FileMgr] Error in the received list item. ";
    if (file_item) {
        LOGE << "[FileMgr] File type: " << (uint8_t)file_item->fileType;
    }
    return kErrorSystemError;
}

int ListTransferRequest::ConsumeSpeakerAudioItem(dji::sdk::ListDataItem *item, uint8_t *data) {
    if (item == nullptr || data == nullptr) {
        return kErrorSystemError;
    }
    std::shared_ptr<SpeakerFileListOutputHandler> handler = std::static_pointer_cast<SpeakerFileListOutputHandler>(output_handler_);
    std::shared_ptr<SpeakerAudioFileList> &output = handler->output_;
    if (output == nullptr) {
        output = std::make_shared<SpeakerAudioFileList>();
        //        output->hasInvalidFile = false;LL
    }
    std::shared_ptr<SpeakerAudioFileInfo> file_item = RemoteFileHelper::GetSpeakerAudioFileHelper(*item, data);

    if (file_item) {
        output->fileList.push_back(*file_item.get());
        return kNoError;
    }

    //    output->hasInvalidFile = true;
    LOGE << "Error in the received list item. ";

    return kErrorSystemError;
}

int ListTransferRequest::ConsumeMediaItem(ListDataItem *item, uint8_t *data) {
    if (item == nullptr || data == nullptr) {
        return kErrorSystemError;
    }
    std::shared_ptr<FileListOutputHandler> handler = std::static_pointer_cast<FileListOutputHandler>(output_handler_);
    std::shared_ptr<FileList> &output = handler->output_;
    if (output == nullptr) {
        output = std::make_shared<FileList>();
        output->hasInvalidFile = false;
        output->files.type = FileType::MEDIA;
    }

    std::shared_ptr<MediaFile> file_item = RemoteFileHelper::GetFileHelper(*item, camera_type_, data);

    if (file_item && (uint8_t)file_item->fileType != 0xff) {
        // LOGD << "ListTransferRequest::ConsumeMediaItem receive file item: index -> " <<  (int)file_item->fileIndex << " GroupId -> " <<
        // (int)file_item->fileGroupIndex << " star -> " << (int)file_item->starTag;
        // Todo: Refactor it with strategy
        // TODO: Jayme上面那句话以后上层来决定是否成组，方法还未考虑，Class内部暂时使用的是group_file_perform_

        //--------------------------------------------------------------------------------------------
        // MARK: 全景图片
        // 相机存的是个html，照片用另外一种命名方式存在PANO目录，所以需要成组
        // 相机传来的是一个文件的数据结构（file_item），需要根据PanoCount来设定subIndex
        // 生成的SubMediaFile文件的FileIndex相同，SubIndex不同
        //--------------------------------------------------------------------------------------------
        if (file_item->fileType == MediaFileType::PANORAMA) {
            // Group file handle, Group operation is handled by firmware, such as PANO
            for (int i = 1; i <= file_item->panoCount; i++) {
                MediaFile subPanoFile;
                subPanoFile.fileIndex = file_item->fileIndex;
                subPanoFile.subIndex = i;
                file_item->subMediaFile.push_back(subPanoFile);
            }
            output->files.media.push_back(*file_item.get());
            return kNoError;
        }
        //--------------------------------------------------------------------------------------------
        // MARK: AEB, Burst, Interval
        // 相机会连续储存一系列Index不同、但是GroupIndex相同的独立文件 （65332, 1）
        // GroupIndex存在重复的情况 (V1协议定的垃圾)
        // (Index, GroupIndex) : （65332, 1）（65333, 1）（65334, 1）（65335, 0）（65336, 2）（65337, 2）（65338, 1）（65339, 1）
        // 以上这种情况
        // 65332--65334 为一组A
        // 65335 是单独非成组文件
        // 65336--65337 为一组B
        // 65338--65339 为一组C，其GroupIndex和A组重复
        //
        // 回放组拉取时 SDK 不进行整组工作，组拉取会将组文件总数填进 pano_count
        //--------------------------------------------------------------------------------------------
        bool group_index_exist = file_item->fileGroupIndex != 0;
        bool is_multiple_pic =
            (file_item->photoType == MediaPhotoType::AEB || file_item->photoType == MediaPhotoType::BURST || file_item->photoType == MediaPhotoType::INTERVAL);
        bool group_file_perform = camera_type_ == DJI_CAMERA_TYPE_AC101 || camera_type_ == DJI_CAMERA_TYPE_HG200;
        bool non_group_fetch_result = file_item->panoCount == 0 ? true : false;
        if (group_file_perform && group_index_exist && is_multiple_pic && non_group_fetch_result) {
            // 保证加入SubMediaFile时已经存在一个文件在output
            // 解决GroupIndex连续判断
            if (!output->files.media.empty() && output->files.media.back().fileGroupIndex == file_item->fileGroupIndex) {
                // Group file with same groupId, Group operation is handled by SDK or APP, such as AEB/BURST
                MediaFile &manual_group_item = output->files.media.back();
                manual_group_item.subMediaFile.push_back(*file_item.get());
            } else {
                MediaFile &manual_group_item = *file_item.get();
                MediaFile sub_media_file = *file_item.get();
                // 生成虚拟文件，并且放入第一个文件
                manual_group_item.isManualGroupFile = true;
                manual_group_item.subMediaFile.push_back(sub_media_file);
                output->files.media.push_back(manual_group_item);
            }
            return kNoError;
        }
        if (group_file_perform && is_multiple_pic) {
            // 组拉取时 pano_count > 1，标记为 manual group
            file_item->isManualGroupFile = (file_item->panoCount > 1 ? true : false);
        }

        //--------------------------------------------------------------------------------------------
        // MARK: Video 4G Segment
        // 在FAT32系统里最大只能存4G的文件，如果录制视频大于4G，会分割成一系列 Index 相同，
        // SegmentSubIndex 从 0 开始的独立文件
        // Index生成如下
        // (Index, SegmentIndex) : (62211, 0) (62212,0) (62212, 1) (62212, 2) (62213, 0)
        // 以上这种情况
        // 62211、62213 是正常普通的文件
        // 62212 是一个被分成3段的视频
        // 其中 （62212, 0） 会被当作普通文件处理， 1, 2分段会被这里截获
        //
        // 回放组拉取时固件填写分段数量到 pano_count，同时统计总时长到 duration
        // 由目前分段归组的逻辑来看，无需另做特殊处理
        //--------------------------------------------------------------------------------------------
        bool segment_exist = file_item->segSubIndex != 0;
        bool is_video = (file_item->fileType == MediaFileType::MOV || file_item->fileType == MediaFileType::MP4);
        // 需要判断 !output->files.empty()
        // 是因为如果我仅有一个分段的视频在SD卡内
        // 第一个文件还是要走普通文件逻辑
        if (group_file_perform && !output->files.media.empty() && segment_exist && is_video) {
            // 拥有相同的Index
            if (output->files.media.back().fileIndex == file_item->fileIndex) {
                MediaFile &manual_group_item = output->files.media.back();
                //存在Seg1，但是第一个 Seg0 本身还未被加入 subMediaFile
                if (manual_group_item.subMediaFile.empty()) {
                    // Copy 一份 并且放入第一个Segment文件
                    MediaFile seg0 = manual_group_item;
                    manual_group_item.subMediaFile.push_back(seg0);
                    manual_group_item.isManualGroupFile = true;
                }
                MediaFile seg1 = *file_item.get();
                //放入Segment文件并且拼接时长
                manual_group_item.subMediaFile.push_back(seg1);
                manual_group_item.duration += seg1.duration;
                manual_group_item.fileSize += seg1.fileSize;
                return kNoError;
            } else {
                //如果 和上一个fileIndex不相同 但是 SegSubIndex 又不为0
                // 210和101的策略那么这个视频就读不出来了
                //但是！！
                // SDK把其作为普通文件，并且为后续的Seg文件做Seg0 文件
                //比如用户手贱插入SD卡，就仅仅删除(62212,0) 这个文件
                //
            }
        }
        //--------------------------------------------------------------------------------------------
        // MARK: 正常情况,直接放入文件
        //--------------------------------------------------------------------------------------------
        output->files.media.push_back(*file_item.get());
        return kNoError;

        //--------------------------------------------------------------------------------------------
        // MARK: 后续的坑
        // MARK: 后续的坑
        // MARK: 后续的坑
        //
        // AEB, Burst, Interval, Video 4G Segment
        // 我们假定这些Segment文件是连续的
        // 因为目前没有机器能在录视频的时候插入一个照片
        // (62212, 0) (62212, 1) (62213, 0) (62212, 2)
        // 不会出现这种储存情况
        // 但是后续的双相机中，这种情况肯定存在，需要重新定协议
        // ！！！！
        // ！！！！
        // 定协议的时候请把GroupIndex定的不要重复，并且一次性兼容全景、Video、AEB这三种解析情况
        // 反正估计也不是我定，当你看到这行注释说明你又被硬件坑了
        // ！！！！
        // ！！！！
        //--------------------------------------------------------------------------------------------
    }
    output->hasInvalidFile = true;
    LOGE << "[FileMgr] Error in the received list item. ";
    if (file_item) {
        LOGE << "[FileMgr] File type: " << (uint8_t)file_item->fileType << "index: " << file_item->fileIndex;
    }
    return kErrorSystemError;
}

void ListTransferRequest::ResetInternalData() {
    for (auto &data : data_buffers_) {
        free(data.first);
    }
    data_buffers_.clear();
    parse_state_ = ParseState::INITIAL;
    if (header_ != nullptr) {
        free(header_);
        header_ = nullptr;
    }

    if (item_header_ != nullptr) {
        free(item_header_);
        item_header_ = nullptr;
    }

    output_handler_->ResetOutput();
}

std::string ListTransferRequest::GetDescription() {
    return "File request session: " + std::to_string(GetSessionID());
}

int ListTransferRequest::GetTimeoutDuration() {
    return 10000;
}

int ListTransferRequest::GetAbortSessionTaskID() {
    return DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST;
}

