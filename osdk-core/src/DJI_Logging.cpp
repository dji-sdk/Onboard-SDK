/** @file DJI_Logging.cpp
 *  @version 3.1.9
 *  @date August 15th, 2016
 *
 *  @brief
 *  Implement logging functions for the SDK.
 *
 *  @copyright 2016 DJI. All right reserved.
 *
 */

#include "DJI_Logging.h"
#include "DJI_API.h"
#include "DJI_Link.h"
#include "DJI_Type.h"
#ifdef API_TRACE_DATA
#include <pthread.h>

namespace DJI {
namespace onboardSDK {

pthread_mutex_t _logging_lock = PTHREAD_MUTEX_INITIALIZER;

void printFrame(HardDriver *serialDevice, Header *header, bool onboardToAircraft) {
    pthread_mutex_lock(&_logging_lock);

  uint32_t *crc32 = (uint32_t *)((uint8_t *)header + header->length - 4);

  if (!header->isAck) {
    __Command *command = (__Command *)((uint8_t *)header + sizeof(Header));

    if (!onboardToAircraft && command->set_id == SET_BROADCAST) {
      pthread_mutex_unlock(&_logging_lock);
      return;
    }

        API_LOG(serialDevice, DEBUG_LOG, "\n\n");
        if (onboardToAircraft) {
            API_LOG(serialDevice, DEBUG_LOG, "|---------------------Sending To Aircraft-------------------------------------------------------------|\n");
        } else {
            API_LOG(serialDevice, DEBUG_LOG, "|---------------------Received From Aircraft-----------------------------------------------------------|\n");
        }

        API_LOG(serialDevice, DEBUG_LOG,
                "|<---------------------Header-------------------------------->|<---CMD frame data--->|<--Tail-->|\n");
        API_LOG(serialDevice, DEBUG_LOG,
                "|SOF |LEN |VER|SESSION|ACK|RES0|PADDING|ENC|RES1|SEQ   |CRC16 |CMD SET|CMD ID|CMD VAL|  CRC32   |\n");
        API_LOG(serialDevice, DEBUG_LOG,
                "|0x%2X|%4d|%3d|%7d|%3d|%4d|%7d|%3d|%4d|%6d|0x%04X|  0x%02X | 0x%02X |       |0x%08X|\n", header->sof,
                header->length, header->version, header->sessionID, header->isAck,
                header->reversed0, header->padding, header->enc, header->reversed1,
                header->sequenceNumber, header->crc, command->set_id, command->id, *crc32);

        if (command->set_id == SET_ACTIVATION && command->id == 0x00) {
//            API_LOG(serialDevice, DEBUG_LOG,
//                    "command\tset: %d %s\ncommand id:");
//        __ActivationGetProtocolVersionCommand aCommand = (__ActivationGetProtocolVersionCommand) &command;
        }
    } else {
        API_LOG(serialDevice, DEBUG_LOG, "\n\n");
        if (onboardToAircraft) {
            API_LOG(serialDevice, DEBUG_LOG, "|---------------------Sending To Aircraft-------------------------------------------------------------|\n");
        } else {
            API_LOG(serialDevice, DEBUG_LOG, "|---------------------Received From Aircraft-----------------------------------------------------------|\n");
        }

        API_LOG(serialDevice, DEBUG_LOG,
                "|<---------------------Header-------------------------------->|<-ACK frame data->|<--Tail-->|\n");
        API_LOG(serialDevice, DEBUG_LOG,
                "|SOF |LEN |VER|SESSION|ACK|RES0|PADDING|ENC|RES1|SEQ   |CRC16 |      ACK VAL     |  CRC32   |\n");
        API_LOG(serialDevice, DEBUG_LOG,
                "|0x%2X|%4d|%3d|%7d|%3d|%4d|%7d|%3d|%4d|%6d|0x%04X|      ACK VAL     |0x%08X|\n", header->sof,
                header->length, header->version, header->sessionID, header->isAck,
                header->reversed0, header->padding, header->enc, header->reversed1,
                header->sequenceNumber, header->crc, *crc32);
    }
    pthread_mutex_unlock(&_logging_lock);
}
}
}
#endif  // API_TRACE_DATA
