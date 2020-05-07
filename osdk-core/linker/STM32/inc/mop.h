/*************************************************************************
 > File Name: mop.h
 > Author: dafeng.xu
 > Created Time: Mon 09 Dec 2019 12:09:58 PM CST
************************************************************************/

#ifndef __MOP_H__
#define __MOP_H__

#include "mop_osal.h"

typedef void * mop_channel_handle_t;

#define MOP_SUCCESS                   (0)
#define MOP_ERR_FAILED                (-10001)
#define MOP_ERR_CRC                   (-10002)
#define MOP_ERR_PARM                  (-10003)
#define MOP_ERR_NOMEM                 (-10004)
#define MOP_ERR_NOTREADY              (-10005)
#define MOP_ERR_SEND                  (-10006)
#define MOP_ERR_RECV                  (-10007)
#define MOP_ERR_TIMEOUT               (-10008)
#define MOP_ERR_RESBUSY               (-10009)
#define MOP_ERR_RESOCCUPIED           (-10010)
#define MOP_ERR_CONNECTIONCLOSE       (-10011)
#define MOP_ERR_CONNECTING            (-10012)
#define MOP_ERR_NORESOURSE            (-10013)
#define MOP_ERR_CLOSING               (-10014)
#define MOP_ERR_NOTCONNECT            (-10015)
#define MOP_ERR_LINKDISCONNECT        (-10016)
#define MOP_ERR_CONNECTREJECT         (-10017)
#define MOP_ERR_HASBINDED             (-10018)
#define MOP_ERR_STATEWRONG            (-10019)
#define MOP_ERR_NOTIMPLEMENT          (-10100)


#define MOP_WAIT_FOREVER        (uint32_t)(-1)
#define MOP_WAIT_POLLING        (uint32_t)(0)

typedef enum {
    MOP_DEVICE_ROUTER = 0,
    MOP_DEVICE_MSDK,
    MOP_DEVICE_PSDK,
    MOP_DEVICE_OSDK
}mop_device_t;

typedef enum {
    MOP_TRANS_RELIABLE,
    MOP_TRANS_UNRELIABLE
}mop_trans_t;

#ifdef __cplusplus
extern "C" {
#endif

int32_t mop_create_channel(mop_channel_handle_t *chl_handle, mop_trans_t trans);
int32_t mop_destroy_channel(mop_channel_handle_t chl_handle);

int32_t mop_bind_channel(mop_channel_handle_t chl_handle, uint16_t channel_id);

int32_t mop_connect_channel(mop_channel_handle_t chl_handle,
                           mop_device_t device,
                           uint8_t slot,
                           uint16_t channel_id);
int32_t mop_accept_channel(mop_channel_handle_t chl_handle,
                          mop_channel_handle_t *out_chl_handle);

int32_t mop_read_channel(mop_channel_handle_t chl_handle,
                        void *buf, uint32_t length);
int32_t mop_write_channel(mop_channel_handle_t chl_handle,
                         void *buf, uint32_t length);

int32_t mop_close_channel(mop_channel_handle_t chl_handle);

int32_t mop_set_channel_opt(mop_channel_handle_t chl_handle);
int32_t mop_add_multicast(mop_device_t device);

#ifdef __cplusplus
}
#endif
#endif