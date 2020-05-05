//
//  commondatarangehandler.cpp
//  djisdk
//
//  Created by husong on 1/14/18.
//
//
#include "commondatarangehandler.h"
#include <stdio.h>

namespace DJI {
namespace OSDK {
std::vector<Range>& CommonDataRangeHandler::GetNoAckRanges() {
    return m_noAckRanges;
}

uint32_t CommonDataRangeHandler::GetLastNotReceiveSeq() {
    return m_lastNotReceivedSeq;
}

bool CommonDataRangeHandler::IsResentAllNeeded() {
    return m_is_resent_all;
}

void CommonDataRangeHandler::DeInit() {
  m_noAckRanges.clear();
  m_lastNotReceivedSeq = 0;
  m_is_resent_all = false;
}

void CommonDataRangeHandler::AddSeqIndex(uint32_t seqNum, uint32_t confirmSeq, uint32_t bufSize) {
    do {
        if (seqNum > confirmSeq + bufSize) {
            // Logic should not go to here, because buffering seq will return false if (seqNum > confirmSeq + bufSize)
            break;
        } else {
            if (seqNum == m_lastNotReceivedSeq) {
                m_lastNotReceivedSeq++;
                break;
            } else if (seqNum > m_lastNotReceivedSeq) {
                Range range = {m_lastNotReceivedSeq, seqNum - m_lastNotReceivedSeq};
                m_noAckRanges.push_back(range);
                m_lastNotReceivedSeq = seqNum + 1;
                break;
            } else {
                // 小于当前期待收包seq，看是否在需要重传range之中
                for (auto&& range = m_noAckRanges.begin(); range < m_noAckRanges.end(); ++range) {
                    if (seqNum < range->seq_num) {
                        // 小于当前Range最小值，直接忽略
                        continue;
                    } else if (range->seq_num == seqNum) {
                        // 等于当前Range最小值
                        if (range->length <= 1) {
                            m_noAckRanges.erase(range);
                            break;
                        }
                        range->seq_num = seqNum + 1;
                        range->length = range->length - 1;
                        break;
                    } else if (seqNum > range->seq_num && seqNum < range->seq_num + range->length - 1) {
                        // 在当前Range中间位置，拆分为两个Range
                        Range rangeLeft = {range->seq_num, seqNum - range->seq_num};
                        Range rangeRight = {seqNum + 1, range->seq_num + range->length - seqNum - 1};
                        *range = rangeLeft;
                        m_noAckRanges.insert(range, rangeRight);
                        break;
                    } else if (range->seq_num + range->length - 1 == seqNum) {
                        // 等于当前Range最大值
                        if (range->length <= 1) {
                            m_noAckRanges.erase(range);
                            break;
                        }
                        range->length = range->length - 1;
                        break;
                    } else {
                        // 大于当前Range
                        continue;
                    }
                }
            }
        }
    } while (false);
}
}  // namespace OSDK

}  // namespace DJI
