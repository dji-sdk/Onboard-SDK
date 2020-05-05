/*************************************************************************
 > File Name: mop_entry_osdk.c
 > Author: dafeng.xu
 > Email: dafeng.xu@dji.com
 > Created Time: Thu 12 Dec 2019 03:10:50 PM CST
************************************************************************/

#include "mop_osal.h"
#include "mop_link_layer.h"
#include "mop_net_layer.h"
#include "mop_priv.h"
#include "mop_buffer.h"
#include "mop_trans_layer.h"
#include "mop_entry_osdk.h"
#include "osdk_command.h"
#include "osdk_command_instance.h"

#define MAX_BACKLOG                  (20)
#define ASSEMBLE_LEN                 (32768)
#define MOP_RECV_BUFFER_MAX_NUM      (8)
#define MOP_RECV_BUFFER_LENGTH       (32*1024)

typedef struct {
    void *send_buf;
    void *recv_buf;
    uint32_t send_len;
    uint32_t recv_len;
} link_buffer;

typedef struct {
    bool flag;
    void *buffer;
    uint32_t length;
} __attribute__((packed)) T_recvBufferItem;

typedef struct {
    T_recvBufferItem bufferItem[MOP_RECV_BUFFER_MAX_NUM];
    int writeIdx;
    int readIdx;
} __attribute__((packed)) T_mopRecvCtx;

static link_buffer assemble_buf = {NULL, NULL, 0, 0};

static E_OsdkStat mopRecvHandler(struct _CommandHandle *cmdHandle,
                                 const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData);

static T_mopRecvCtx s_recvCtx;
static mop_route_item_t o2e1e_slv_route_tab[] = {{NULL, MOP_DEVICE_ROUTER, 1},
                                                 {NULL, MOP_DEVICE_ROUTER, 0},
                                                 {NULL, MOP_DEVICE_PSDK,   0},
                                                 {NULL, MOP_DEVICE_PSDK,   1},
                                                 {NULL, MOP_DEVICE_PSDK,   2},
                                                 {NULL, MOP_DEVICE_MSDK,   0},
                                                 {NULL, MOP_DEVICE_MSDK,   1}};

static T_RecvCmdItem s_bulkCmdList[] = {
    PROT_CMD_ITEM(0, 0, 0x66, 0x01, MASK_HOST_DEVICE_SET_ID, NULL,
                  mopRecvHandler),
};

E_OsdkStat mopRecvHandler(struct _CommandHandle *cmdHandle,
                          const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData)
{
    int count = 0, i;
    static int index = 1;
    i = s_recvCtx.writeIdx;

    while (s_recvCtx.bufferItem[i].flag != 0 && count < 5) {
        count++;
        OsdkOsal_TaskSleepMs(4);
    }
    if (count == 5) {
        index++;
        MOP_ENTRY_LOGE("can't get buffer to write data, length %d", cmdInfo->dataLen);
        return OSDK_STAT_ERR;
    }
    if (cmdInfo->dataLen > MOP_RECV_BUFFER_LENGTH) {
        index++;
        MOP_ENTRY_LOGE("recv data is too large");
        return OSDK_STAT_ERR;
    }

    mop_packet_head_t *head = (mop_packet_head_t *) cmdData;
#if 0
    printf("seq num = %u\n", head->seq_num);
    printf("trans type = %x\n", head->trans_type);
    printf("data = %d %d %d\n", head->data[0], head->data[1], head->data[2]);
    printf("start to memcpy, dataLen = %d, index = %d!\n", cmdInfo->dataLen, index);
#endif
    memcpy(s_recvCtx.bufferItem[i].buffer, cmdData, cmdInfo->dataLen);
    s_recvCtx.bufferItem[i].length = cmdInfo->dataLen;
    s_recvCtx.bufferItem[i].flag = 1;
    s_recvCtx.writeIdx = (i + 1) % MOP_RECV_BUFFER_MAX_NUM;
    index++;
    return OSDK_STAT_OK;
}

static int32_t osdk_check_liveview_usb_connection()
{
    E_OsdkStat osdkStat;
    osdkStat = OsdkCommand_CheckLiveViewChannel();
    if (osdkStat == OSDK_STAT_OK) {
        return MOP_SUCCESS;
    }
    return MOP_ERR_FAILED;
}

static int32_t osdk_notify_slave()
{
    T_CmdInfo info;
    T_CmdInfo ackInfo;
    uint8_t ackData[1024];
    int result;
    uint8_t data = 0;

    info.cmdSet = 0x49;
    info.cmdId = 0x30;
    info.dataLen = 1;
    info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
    info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
    info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
    info.encType = 0;
    info.sender = 0xca;
    info.receiver = 0x07;
    result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, &data,
                                  &ackInfo, ackData, 2000, 3);

    if (result != OSDK_STAT_OK) {
        if (result == OSDK_STAT_ERR_TIMEOUT) {
            MOP_ENTRY_LOGE("timeout, no ack receive!\n");
        }
        return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

static int32_t send_bulk_data(void *plink, void *buf, uint16_t length, uint8_t immediate)
{
    E_OsdkStat osdkStat;
    T_CmdInfo sendInfo;
    int ret = 0;
    mop_link_layer_node_t *plink_node = NULL;
    uint32_t data_len = 0;
    mop_packet_head_t *packet_head;
    static int err_count = 0;

    if (length > 0) {
        memcpy((void *) ((uint8_t *) assemble_buf.send_buf + assemble_buf.send_len), buf, length);
        assemble_buf.send_len += MOP_PACKET_LENGTH;
    } else if (assemble_buf.send_len == 0) {
        return length;
    }

    if ((immediate == 0) && (assemble_buf.send_len <= (ASSEMBLE_LEN - MOP_PACKET_LENGTH))) {
        return length;
    }

//send_buf
    sendInfo.cmdSet = 0x66;
    sendInfo.cmdId = 0x01;
    sendInfo.dataLen = assemble_buf.send_len;
    sendInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
    sendInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
    sendInfo.addr = GEN_ADDR(0, ADDR_BIGDATA_LIVEVIEW_INDEX);
    sendInfo.encType = 0;

    MOP_ENTRY_LOGE("start send bulk data, len = %d, err_cnt = %d", assemble_buf.send_len, err_count);
    osdkStat = OsdkCommand_BigDataSend(&sendInfo, (uint8_t *) assemble_buf.send_buf);
    if (osdkStat != OSDK_STAT_OK) {
        MOP_ENTRY_LOGE("send bulk data failed");
        err_count++;
    }
    assemble_buf.send_len = 0;
    memset(assemble_buf.send_buf, 0, ASSEMBLE_LEN);
    return length;
}

static int32_t recv_bulk_data(void *plink, void *buf, uint16_t plength)
{
    int count = 0, i = 0;
    static int offset = 0;
    i = s_recvCtx.readIdx;
    while (s_recvCtx.bufferItem[i].flag != 1 && count < 50) {
        count++;
        OsdkOsal_TaskSleepMs(4);
    }
    if (count == 50) {
        //MOP_ENTRY_LOGE("no data to read"); MOP_ENTRY_LOGE("recv data = %d %d %d %d",
        return 0;
    }

    if (s_recvCtx.bufferItem[i].length >= plength) {
        memcpy(buf, s_recvCtx.bufferItem[i].buffer + offset, plength);
        s_recvCtx.bufferItem[i].length -= plength;
        offset += plength;
    } else {
        memcpy(buf, s_recvCtx.bufferItem[i].buffer + offset, s_recvCtx.bufferItem[i].length);
        s_recvCtx.bufferItem[i].length = 0;
        offset = 0;
    }

    if (s_recvCtx.bufferItem[i].length == 0) {
        offset = 0;
        s_recvCtx.bufferItem[i].flag = 0;
        s_recvCtx.readIdx = (i + 1) % MOP_RECV_BUFFER_MAX_NUM;
    }

    return plength;
}

static int32_t osdk_create_e1e_slave_connection(mop_link_layer_node_t *plink)
{

    int32_t ret = 0;

    //First step: init recv ctx.
    s_recvCtx.readIdx = 0;
    s_recvCtx.writeIdx = 0;
    for (int i = 0; i < MOP_RECV_BUFFER_MAX_NUM; i++) {
        s_recvCtx.bufferItem[i].flag = 0;
        s_recvCtx.bufferItem[i].length = 0;
        s_recvCtx.bufferItem[i].buffer = OsdkOsal_Malloc(MOP_RECV_BUFFER_LENGTH);
        memset(s_recvCtx.bufferItem[i].buffer, 0, MOP_RECV_BUFFER_LENGTH);
    }

    //Second step: register recv handle func.
    T_RecvCmdHandle recvCmdHandle;
    recvCmdHandle.cmdList = s_bulkCmdList;
    recvCmdHandle.cmdCount = sizeof(s_bulkCmdList) / sizeof(T_RecvCmdItem);
    recvCmdHandle.protoType = PROTOCOL_USBMC;
    ret = OsdkCommand_RegRecvCmdHandler(OsdkCommand_GetInstance(), &recvCmdHandle);
    if (ret != OSDK_STAT_OK) {
        MOP_ENTRY_LOGE("register cmd handler failed, ret = %d\n", ret);
        return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

int32_t bulk_close(void *plink)
{
    return 0;
}

int32_t bulk_get_link_bandwidth(void *plink, uint32_t *bandwidth_limit_bytes)
{
    *bandwidth_limit_bytes = 1000000;
    return MOP_SUCCESS;
}

int32_t init_assemble_buffer()
{
    assemble_buf.send_buf = mop_osal_malloc(ASSEMBLE_LEN);
    if (assemble_buf.send_buf == NULL) {
        MOP_ENTRY_LOGE("malloc send assemble buffer failed");
        return MOP_ERR_FAILED;
    }
    assemble_buf.send_len = 0;
    assemble_buf.recv_buf = mop_osal_malloc(ASSEMBLE_LEN);
    if (assemble_buf.recv_buf == NULL) {
        MOP_ENTRY_LOGE("malloc recv assemble buffer failed");
        mop_osal_free(assemble_buf.send_buf);
        return MOP_ERR_FAILED;
    }
    assemble_buf.recv_len = 0;
    return MOP_SUCCESS;
}

int32_t destroy_assemble_buffer()
{
    mop_osal_free(assemble_buf.send_buf);
    mop_osal_free(assemble_buf.recv_buf);
    assemble_buf.send_len = 0;
    assemble_buf.recv_len = 0;
    return MOP_SUCCESS;
}

int32_t mop_check_hardware_status(void *plink)
{

    //First step: check usb connection.
    int32_t ret = 0;
    plink = plink;

    ret = osdk_check_liveview_usb_connection();
    if (MOP_SUCCESS != ret) {
        MOP_ENTRY_LOGE("connection check failed");
        return ret;
    }

    if (osdk_notify_slave() != MOP_SUCCESS) {
        return MOP_ERR_FAILED;
    }
    return MOP_SUCCESS;
}

//@TODO
int32_t mop_osdk_deinit()
{
    mop_trans_layer_stop_task();
    mop_net_layer_stop_task();
    mop_net_layer_release_resource();
    mop_trans_layer_release_resource();
    mop_deinit_buffer();

    //release resources here
    destroy_assemble_buffer();

    return MOP_SUCCESS;
}

int32_t mop_osdk_init()
{
    int32_t ret = 0;
    mop_link_layer_node_t *plink_node_to_slave = (mop_link_layer_node_t *) (0);

    ret = mop_link_layer_node_init(&plink_node_to_slave, MOP_LINK_USBBULK, MOP_DEVICE_ROUTER, 1, MOP_DEVICE_OSDK, 0);
    if (MOP_SUCCESS != ret) {
        MOP_ENTRY_LOGE("osdk init link node to slave failed,ret:%d", ret);
        return NULL;
    }
    (plink_node_to_slave->link_layer_ops).send_data = send_bulk_data;
    (plink_node_to_slave->link_layer_ops).recv_data = recv_bulk_data;
    (plink_node_to_slave->link_layer_ops).check_hw_status = mop_check_hardware_status;
    (plink_node_to_slave->link_layer_ops).link_reconnect = osdk_create_e1e_slave_connection;
    (plink_node_to_slave->link_layer_ops).close = bulk_close;
    (plink_node_to_slave->link_layer_ops).get_link_bandwidth  = bulk_get_link_bandwidth;
    plink_node_to_slave->route_table = (mop_route_item_t *) o2e1e_slv_route_tab;
    plink_node_to_slave->table_length = sizeof(o2e1e_slv_route_tab) / sizeof(o2e1e_slv_route_tab[0]);

    init_assemble_buffer();

    ret = mop_init_buffer();
    if (MOP_SUCCESS != ret) {
        MOP_ENTRY_LOGE("init buffer failed,ret:%d", ret);
        return MOP_ERR_FAILED;
    }

    ret = mop_net_layer_init(MOP_DEVICE_OSDK, 0);
    if (MOP_SUCCESS != ret) {
        MOP_ENTRY_LOGE("osdk init net layer failed,ret:%d", ret);
        return MOP_ERR_FAILED;
    }

    ret = mop_trans_layer_init();
    if (ret != MOP_SUCCESS) {
        MOP_ENTRY_LOGE("trans layer init failed, ret = %d", ret);
        return MOP_ERR_FAILED;
    }
    mop_trans_layer_start();

    mop_link_layer_connect(plink_node_to_slave);

    sleep(1);

    return MOP_SUCCESS;
}
