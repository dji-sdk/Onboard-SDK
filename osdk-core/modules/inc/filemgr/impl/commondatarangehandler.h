//
//  commondatarangehandler.h
//  djisdk
//
//  Created by husong on 1/14/18.
//
//

#ifndef commondatarangehandler_h
#define commondatarangehandler_h
#include <vector>
#include <cstdint>

namespace DJI {
namespace OSDK {
struct Range {
    uint32_t seq_num;
    uint32_t length;
};

class CommonDataRangeHandler final {
public:
    void DeInit();
    // seqNum: 收到包seq
    // confirmSeq: 确认收到最大连续包seq
    // bufSize: 缓存buf的大小
    void AddSeqIndex(uint32_t seqNum, uint32_t confirmSeq, uint32_t bufSize);

    bool IsResentAllNeeded();

    std::vector<Range>& GetNoAckRanges();

    uint32_t GetLastNotReceiveSeq();

private:
    std::vector<Range> m_noAckRanges;
    uint32_t m_lastNotReceivedSeq = 0;
    bool m_is_resent_all = false;
};

}  // namespace OSDK

}  // namespace DJI
#endif /* commondatarangehandler_h */
