/**
 ********************************************************************
 * @file    dji_common_define.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   This is the header file for defining the structure and enum
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DJI_COMMON_DEFINE_H
#define DJI_COMMON_DEFINE_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
  OSDK_COMMON_CMD_ID_SIMPLE_TIME_SYNC = 0x58,
} E_DJIExtendCmdId;

typedef enum {
  DJI_SYN_REQUEST = 1,             // 同步请求
  DJI_FOLLOW_REQUEST = 2,          // 跟随请求
  DJI_DELAY_COMPUTER_REQUEST = 3,  // 延迟测算请求
  DJI_MAIN_CLOCK_TIME_PUSH = 4,    // 主时钟时间推送
} DJI_COMMON_SIMPLE_TIME_SYN;

#pragma pack(1)
typedef struct {
  // 子命令ID
  uint8_t sub_command_id;  // enum-type: DJI_COMMON_SIMPLE_TIME_SYN
  // 甲方汇报自身的时间
  uint32_t timestamp;
} dji_general_get_simple_time_sync_req;
typedef struct {
  // 返回码
  uint8_t ret_code;
  // 收到的甲方发来的甲方时间
  uint32_t remote_time;
  // 乙方收到该协议时的乙方时间
  uint32_t recv_time;
  // 乙方应答该协议时的乙方时间
  uint32_t send_time;
} dji_general_get_simple_time_sync_ack;
#pragma pack()
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif  // DJI_COMMON_DEFINE_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
