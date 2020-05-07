//
//  newfiledatarequest.h
//  djisdk
//
//  Created by Xietong LU on 2018/6/10.
//

#ifndef newfiledatarequest_h
#define newfiledatarequest_h

#include <stdio.h>
#include "transferrequest.h"

namespace DJI {
namespace OSDK {
class CommonDataRangeHandler;

class DataTransferRequest : public TransferRequest {
public:
    DataTransferRequest(uint16_t session_id, std::shared_ptr<const FileDataRequest> config, FileDataCallback callback);
    virtual ~DataTransferRequest();

    void SetPatchData(bool patched) {
        patch_data_ = patched;
    }

    virtual dji::core::dji_cmd_req CreateStartRequestPack() override;
    virtual void OnHandlerFailure(int err_code) override;
    virtual void ResetInternalData() override;
    virtual int GetAbortSessionTaskID() override;
    virtual std::string GetDescription() override;

protected:
    int ValidateRequestConfig();
    virtual bool ShouldParseData() override;
    virtual void ParseData(bool is_last_pack, ParsingResultHandler handler) override;

    void TriggerCallback(int err_code, bool is_end, double bit_per_second, const Dji::Common::Buffer& data_buf);

private:
    DJI_DOWNLOAD_FILE_SUBTYPE type_ = DJI_DOWNLOAD_FILE_SUBTYPE::DJI_DOWNLOAD_FILE_ORG;
    std::shared_ptr<const FileDataRequest> config_;
    FileDataCallback file_data_callback_;
    bool patch_data_ = false;

    std::chrono::system_clock::time_point last_parsing_time_;

    dji::common::RateCalculator rate_calculator_;
    //            uint32_t request_data_size_ = 0; // data size for this request
    //            uint32_t delivery_data_size_ = 0; // size of data that has deliveried to app
};
}  // namespace OSDK
}  // namespace DJI

#endif /* newfiledatarequest_h */
