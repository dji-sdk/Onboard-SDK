/*
 * DJI_Pro_App.cpp
 *
 *  Created on: Sep 8, 2015
 *      Author: wuyuwei
 */

#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"

static unsigned char Pro_Encode_Data[1024];
static unsigned char Pro_Encode_ACK[10];

void DJI_Pro_App_Send_Data(unsigned char session_mode, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,
                   unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout ,int retry_time)
{
	ProSendParameter param;
	unsigned char *ptemp = (unsigned char *)Pro_Encode_Data;
	*ptemp++ = cmd_set;
	*ptemp++ = cmd_id;

	memcpy(Pro_Encode_Data + SET_CMD_SIZE,pdata,len);

	param.ack_callback = ack_callback;
    param.session_mode = session_mode;
	param.length = len + SET_CMD_SIZE;
	param.buf = Pro_Encode_Data;
    param.retry_time = retry_time;

	param.ack_timeout = timeout; 
    param.need_encrypt = is_enc;
	
	Pro_Send_Interface(&param);
}

void DJI_Pro_App_Send_Ack(req_id_t req_id, unsigned char *ack, int len)
{
	ProAckParameter param;

	memcpy(Pro_Encode_ACK,ack,len);
	
	param.session_id = req_id.session_id;
	param.seq_num = req_id.sequence_number;
	param.need_encrypt = req_id.need_encrypt;
	param.buf = Pro_Encode_ACK;
	param.length = len;
	Pro_Ack_Interface(&param);
}

static int DJI_Pro_Create_Thread(void *(* func)(void *), void *arg)
{
    pthread_t A_ARR;

    if(pthread_create(&A_ARR,0,func,arg) != 0)
    {
        return -1;
    }
    return 0;
}

/*
 *  interface: drone status control
 */

static int status_ctrl_lock = -1;
static Command_Result_Notify p_status_ctrl_interface = 0;
static unsigned short status_ctrl_return_code = SDK_ERR_NO_RESPONSE;
static cmd_agency_data_t status_ctrl_cmd_data = {0,0};

static void Save_Status_Ctrl_Return_Code(unsigned short ret_code)
{
    status_ctrl_return_code = ret_code;
}

static unsigned short Get_Status_Ctrl_Return_Code(void)
{
    return status_ctrl_return_code;
}

static void DJI_Pro_Status_Ctrl_CallBack(ProHeader *header)
{
    unsigned short ack_data;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        Save_Status_Ctrl_Return_Code(ack_data);
    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

static void * Status_Ctrl_Thread_Func(void * arg)
{
    unsigned char *cmd = (unsigned char *)arg;
    unsigned cmd_timeout = 100;    //unit is ms
    unsigned retry_time = 3;
    unsigned short ack_data = SDK_ERR_NO_RESPONSE;
    static const char err[3][16] = {{"didn't get ack"},{"timeout"},{"cmd refuse"}};

    while(1)
    {
        status_ctrl_cmd_data.cmd_sequence ++;
        status_ctrl_cmd_data.cmd_data = *cmd;
        Save_Status_Ctrl_Return_Code(SDK_ERR_NO_RESPONSE);
        DJI_Pro_App_Send_Data(2,1,MY_CTRL_CMD_SET, API_CMD_REQUEST,(unsigned char*)&status_ctrl_cmd_data,
                  sizeof(status_ctrl_cmd_data),DJI_Pro_Status_Ctrl_CallBack,cmd_timeout, retry_time);
        /* first stage poll */
        usleep(cmd_timeout * retry_time * 1000);
        ack_data = Get_Status_Ctrl_Return_Code();
        if(ack_data == SDK_ERR_NO_RESPONSE || ack_data == SDK_ERR_COMMAND_NOT_SUPPORTED
              || ack_data == REQ_TIME_OUT || ack_data == REQ_REFUSE)
        {
            printf("%s,line %d,Status Ctrl %d %s,Return 0x%X\n",__func__,__LINE__,
                   status_ctrl_cmd_data.cmd_data,&err[ack_data + 1][0],ack_data);
            if(p_status_ctrl_interface)
            {
                p_status_ctrl_interface(ack_data);
            }
            break;
        }
        else if(ack_data == 0x0002)
        {
             /* second stage poll */
            retry_time = 8;
            while(retry_time --)
            {
                sleep(1);
                DJI_Pro_App_Send_Data(2, 1,MY_CTRL_CMD_SET, API_CMD_STATUS_REQUEST,(unsigned char*)&status_ctrl_cmd_data.cmd_sequence,
                          1,DJI_Pro_Status_Ctrl_CallBack, cmd_timeout, 1);

                usleep(cmd_timeout * 1000);
                ack_data = Get_Status_Ctrl_Return_Code();
                if(ack_data == 0x0003)
                {
                    printf("%s,line %d,Command is running\n",__func__,__LINE__);
                    continue;
                }
                else
                    break;
            }
            if(ack_data == 0x0005)
            {
                /* do some delay here to make sure drone being
                 * in final status before call user notice interface*/
                sleep(5);
                printf("%s,line %d,Status Ctrl Successfully\n",__func__,__LINE__);
            }
            else
            {
               printf("%s,line %d,Status Ctrl Failed,Return 0x%X",__func__,__LINE__,ack_data);
            }

            if(p_status_ctrl_interface)
            {
                p_status_ctrl_interface(ack_data);
            }
            break;
        }
        else
        {
            if(p_status_ctrl_interface)
            {
                p_status_ctrl_interface(ack_data);
            }
            printf("%s,line %d,Unknown ERROR,Return 0x%X",__func__,__LINE__,ack_data);
            break;
        }
    }
    status_ctrl_lock = -1;
    return (void*) NULL;
}

/*
 * cmd: 1->go home;4->take-off;6->landing
 * return:-1->parameter error or previous cmd is not finish,otherwise 0
 */

int DJI_Pro_Status_Ctrl(unsigned char cmd,Command_Result_Notify user_notice_entrance)
{
    static unsigned char cur_cmd = 0;

    if(status_ctrl_lock == 0)
    {
        return -1;
    }
    status_ctrl_lock = 0;

    if(cmd != 1 && cmd != 4 && cmd != 6)
    {
        return -1;
    }
    p_status_ctrl_interface = user_notice_entrance ? user_notice_entrance : 0;
    cur_cmd = cmd;

    if(DJI_Pro_Create_Thread(Status_Ctrl_Thread_Func,&cur_cmd) != 0)
    {
        status_ctrl_lock = 0;
        return -1;
    }
    return 0;
}


/*
 *  interface: get api version
 */
static int get_api_ver_lock = -1;
static Get_API_Version_Notify p_get_api_ver_interface = 0;
static version_query_data_t to_user_version_data;

static void DJI_Pro_Get_API_Version_CallBack(ProHeader *header)
{
    unsigned char *ptemp = (unsigned char *)&header->magic;
    char *ptemp2;
    int count = 31;
    version_query_data_t *p_version_data = &to_user_version_data;
    Get_API_Version_Notify p_temp_interface;

    p_version_data->version_ack = ptemp[0] + (ptemp[1] << 8);
    ptemp += 2;
    p_version_data->version_crc = ptemp[0] + (ptemp[1] << 8) +
                                (ptemp[2] << 16) + (ptemp[3] << 24);
    ptemp += 4;
    ptemp2 = p_version_data->version_name;
    while(*ptemp && count)
    {
        *ptemp2 ++ = (char)*ptemp ++;
        count --;
    }
    *ptemp2 = 0;

    if(p_get_api_ver_interface)
    {
        p_temp_interface = p_get_api_ver_interface;
        p_get_api_ver_interface = 0;
        p_temp_interface(&to_user_version_data);
    }
    else
    {
        printf("%s,version ack=%d\n",__func__,p_version_data->version_ack);
        printf("%s,version crc=0x%X\n",__func__,p_version_data->version_crc);
        printf("%s,version name=%s\n",__func__,p_version_data->version_name);
    }
}

static void * Get_API_Version_Thread_Func(void * arg)
{
    version_query_data_t *p_version_data = (version_query_data_t*)arg;

    unsigned cmd_timeout = 100;    //unit is ms
    unsigned retry_time = 3;
    unsigned char cmd_data = 0;

    DJI_Pro_App_Send_Data(2,1,MY_ACTIVATION_SET, API_VER_QUERY,(unsigned char*)&cmd_data,
              1,DJI_Pro_Get_API_Version_CallBack,cmd_timeout, retry_time);

    usleep((cmd_timeout + 50) * retry_time * 1000);
    /* delay more 50ms to avoid call user notice interface at the same time*/
    if(p_get_api_ver_interface)
    {
        p_get_api_ver_interface(p_version_data);
    }

    get_api_ver_lock = -1;
    return (void*)NULL;
}

int DJI_Pro_Get_API_Version(Get_API_Version_Notify user_notice_entrance)
{
    if(get_api_ver_lock == 0)
    {
        return -1;
    }
    get_api_ver_lock = 0;

    p_get_api_ver_interface = user_notice_entrance ? user_notice_entrance : 0;
    to_user_version_data.version_ack = 0xFFFF;
    to_user_version_data.version_crc = 0x0;
    to_user_version_data.version_name[0] = 0;

    if(DJI_Pro_Create_Thread(Get_API_Version_Thread_Func,&to_user_version_data) != 0)
    {
        get_api_ver_lock = -1;
        return -1;
    }
    return 0;
}

/*
 *  interface: activation interface
 */

static int activate_api_lock = -1;
static Command_Result_Notify p_activate_api_interface = 0;
static activate_data_t from_user_account_data;
static unsigned short to_user_activation_result;

static void DJI_Pro_Activate_API_CallBack(ProHeader *header)
{
    unsigned short ack_data;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        to_user_activation_result = ack_data;
        if(ack_data == SDK_ACTIVATE_NEW_DEVICE)
        {
        }
        else
        {
            if(ack_data == SDK_ACTIVATE_SUCCESS)
            {
                printf("Activation Successfully\n");
                if(from_user_account_data.app_key)
                    Pro_Config_Comm_Encrypt_Key(from_user_account_data.app_key);
            }
            else
            {
               printf("%s,line %d,activate ERR code:0x%X\n",__func__,__LINE__,ack_data);
            }
            if(p_activate_api_interface)
            {
                p_activate_api_interface(ack_data);
            }
            activate_api_lock = -1;
        }

    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
        activate_api_lock = -1;
    }
}

static void * Activate_API_Thread_Func(void * arg)
{
    int retry = 12;
    activate_data_t *temp_data_t = (activate_data_t *)arg;
    from_user_account_data = *temp_data_t;
    while(1)
    {
        DJI_Pro_App_Send_Data( 2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION,
                   (unsigned char*)&from_user_account_data,
                   sizeof(from_user_account_data) - sizeof(char *),
                   DJI_Pro_Activate_API_CallBack,1000,1);

        usleep(50000);
        sleep(1);
        if(to_user_activation_result == SDK_ERR_NO_RESPONSE)
        {
            printf("--- warning line %d ---\n",__LINE__);
            if(p_activate_api_interface)
            {
                p_activate_api_interface(SDK_ERR_NO_RESPONSE);
            }
            activate_api_lock = -1;
            break;
        }
        else if(to_user_activation_result == SDK_ACTIVATE_NEW_DEVICE)
        {
            if(--retry)
            {
                printf("Activate try again\n");
                sleep(1);
                continue;
            }
            else
            {
                printf("--- warning line %d ---\n",__LINE__);
                if(p_activate_api_interface)
                {
                    p_activate_api_interface(SDK_ERR_NO_RESPONSE);
                }
                activate_api_lock = -1;
                break;
            }
        }
        else
        {
            break;
        }
    }
    return (void*)NULL;
}

int DJI_Pro_Activate_API(activate_data_t *p_user_data,
                         Command_Result_Notify user_notice_entrance)
{
    if(activate_api_lock == 0)
    {
        return -1;
    }
    activate_api_lock = 0;
    p_activate_api_interface = user_notice_entrance ? user_notice_entrance : 0;
    to_user_activation_result = SDK_ERR_NO_RESPONSE;
    if(DJI_Pro_Create_Thread(Activate_API_Thread_Func,p_user_data) != 0)
    {
        printf("%s,line %d,ERROR\n",__func__,__LINE__);
        activate_api_lock = -1;
        return -1;
    }
    return 0;
}

static void DJI_Pro_User_Activate_Callback(unsigned short ack)
{
    printf("%s,ack=0x%X\n",__func__,ack);
}

/*
 *  interface: transparent transmission interface
 */

static Command_Result_Notify p_transparent_data_interface = 0;

static void DJI_Pro_Send_To_Mobile_Device_CallBack(ProHeader *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        if(p_transparent_data_interface)
            p_transparent_data_interface(ack_data);
    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Send_To_Mobile_Device(unsigned char *data,unsigned char len,
                                  Command_Result_Notify user_notice_entrance)
{
    if(len > 100)
    {
        return -1;
    }

    p_transparent_data_interface = user_notice_entrance ? user_notice_entrance : 0;
    DJI_Pro_App_Send_Data(2, 0, MY_ACTIVATION_SET, API_TRANSPARENT_DATA_TO_MOBILE,
               data,len,DJI_Pro_Send_To_Mobile_Device_CallBack,500,2);

    return 0;
}

/*
 *  interface: request obtain control interface
 */

static Command_Result_Notify p_control_management_interface = 0;

static void DJI_Pro_Control_Management_CallBack(ProHeader *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        if(p_control_management_interface)
            p_control_management_interface(ack_data);

    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }

    switch(ack_data)
    {
    case 0x0001:
        printf("%s,line %d, release control successfully\n",__func__,__LINE__);
        break;
    case 0x0002:
        printf("%s,line %d, obtain control successfully\n",__func__,__LINE__);
        break;
    case 0x0003:
        printf("%s,line %d, obtain control failed\n",__func__,__LINE__);
        break;
    default:
        printf("%s,line %d, there is unkown error,ack=0x%X\n",__func__,__LINE__,ack_data);
        break;
    }
}

int DJI_Pro_Control_Management(unsigned char cmd,Command_Result_Notify user_notice_entrance)
{
    unsigned char data = cmd & 0x1;
    DJI_Pro_App_Send_Data(2,1, MY_CTRL_CMD_SET, API_CTRL_MANAGEMENT,
               &data,1,NULL,500,1);
    usleep(50000);
    p_control_management_interface = user_notice_entrance ? user_notice_entrance : 0;
    DJI_Pro_App_Send_Data(2,1, MY_CTRL_CMD_SET, API_CTRL_MANAGEMENT,
               &data,1,DJI_Pro_Control_Management_CallBack,500,1);
    return 0;
}

/*
 *  interface: attitude control interface
 */

int DJI_Pro_Attitude_Control(attitude_data_t *p_user_data)
{
    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, API_CTRL_REQUEST,
               (unsigned char *)p_user_data,sizeof(attitude_data_t),
                  0,0,1);
    return 0;
}

/*
 *  interface: gimbal angle control interface
 */
int DJI_Pro_Gimbal_Angle_Control(gimbal_custom_control_angle_t *p_user_data)
{
    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, API_GIMBAL_CTRL_ANGLE_REQUEST,
               (unsigned char *)p_user_data,sizeof(gimbal_custom_control_angle_t),
                  0,0,1);
    return 0;
}

/*
 *  interface: gimbal angle speed control interface
 */
int DJI_Pro_Gimbal_Speed_Control(gimbal_custom_speed_t *p_user_data)
{
    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, API_GIMBAL_CTRL_SPEED_REQUEST,
               (unsigned char *)p_user_data,sizeof(gimbal_custom_speed_t),
                  0,0,1);
    return 0;
}

/*
 *  interface: camera control
 */
int DJI_Pro_Camera_Control(unsigned char camera_cmd)
{
    unsigned char send_data = 0;

    if(camera_cmd != API_CAMERA_SHOT && camera_cmd != API_CAMERA_VIDEO_START
            && camera_cmd != API_CAMERA_VIDEO_STOP)
    {
        printf("%s,line %d,Param ERROR\n",__func__,__LINE__);
        return -1;
    }

    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, camera_cmd,
               (unsigned char*)&send_data,sizeof(send_data),0,0,0);

    return 0;
}

/*
 * interface: get cmd set id
 */
unsigned char DJI_Pro_Get_CmdSet_Id(ProHeader *header)
{
    unsigned char *ptemp = (unsigned char *)&header->magic;
    return *ptemp;
}

/*
 * interface: get cmd code id
 */
unsigned char DJI_Pro_Get_CmdCode_Id(ProHeader *header)
{
    unsigned char *ptemp = (unsigned char *)&header->magic;
    ptemp ++;
    return *ptemp;
}

static sdk_std_msg_t std_broadcast_data;
static pthread_mutex_t std_msg_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned short std_msg_flag = 0;

/*
 * interface: get broadcast data
 */

int DJI_Pro_Get_Broadcast_Data(sdk_std_msg_t *p_user_buf) {
	pthread_mutex_lock(&std_msg_lock); 	
	*p_user_buf = std_broadcast_data; 	
	pthread_mutex_unlock(&std_msg_lock); 	
	return 0; 	
}

int DJI_Pro_Get_Bat_Capacity(unsigned char *p_user_buf)
{
    pthread_mutex_lock(&std_msg_lock);
    *p_user_buf = std_broadcast_data.battery_remaining_capacity;
    pthread_mutex_unlock(&std_msg_lock);
    return 0;
}

int DJI_Pro_Get_Quaternion(api_quaternion_data_t *p_user_buf)
{
    pthread_mutex_lock(&std_msg_lock);
    *p_user_buf = std_broadcast_data.q;
    pthread_mutex_unlock(&std_msg_lock);
    return 0;
}

int DJI_Pro_Get_GroundAcc(api_common_data_t *p_user_buf)
{
    pthread_mutex_lock(&std_msg_lock);
    *p_user_buf = std_broadcast_data.a;
    pthread_mutex_unlock(&std_msg_lock);
    return 0;
}

int DJI_Pro_Get_GroundVo(api_vel_data_t *p_user_buf)
{
    pthread_mutex_lock(&std_msg_lock);
    *p_user_buf = std_broadcast_data.v;
    pthread_mutex_unlock(&std_msg_lock);
    return 0;
}

int DJI_Pro_Get_CtrlInfo(api_ctrl_info_data_t *p_user_buf)
{
    pthread_mutex_lock(&std_msg_lock);
    *p_user_buf = std_broadcast_data.ctrl_info;
    pthread_mutex_unlock(&std_msg_lock);
    return 0;
}

//.... TODO

static void DJI_Pro_Parse_Broadcast_Data(ProHeader *header)
{
    unsigned char *pdata = (unsigned char *)&header->magic;
    unsigned short *msg_enable_flag;
    unsigned short data_len = MSG_ENABLE_FLAG_LEN;
    pthread_mutex_lock(&std_msg_lock);
    pdata += 2;
    msg_enable_flag = (unsigned short *)pdata;
    std_msg_flag = *msg_enable_flag;
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_TIME	,std_broadcast_data.time_stamp      , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_Q		,std_broadcast_data.q				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_A		,std_broadcast_data.a				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_V		,std_broadcast_data.v				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_W		,std_broadcast_data.w				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_POS     ,std_broadcast_data.pos				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_MAG     ,std_broadcast_data.mag				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_RC		,std_broadcast_data.rc				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_GIMBAL	,std_broadcast_data.gimbal			, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_STATUS	,std_broadcast_data.status			, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_BATTERY ,std_broadcast_data.battery_remaining_capacity	, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_DEVICE	,std_broadcast_data.ctrl_info			, pdata, data_len);
    pthread_mutex_unlock(&std_msg_lock);
}

/*
 * interface: protocol initialization
 */
static User_Handler_Func p_user_handler_func = 0;
static Transparent_Transmission_Func p_user_rec_func = 0;
static void DJI_Pro_App_Recv_Req_Data(ProHeader *header)
{
    unsigned char buf[100] = {0,0};
    unsigned char len = 0;
    switch(header->session_id)
    {
    case 0:
        if(DJI_Pro_Get_CmdSet_Id(header) == MY_BROADCAST_CMD_SET
                && DJI_Pro_Get_CmdCode_Id(header) == API_STD_DATA)
        {
            DJI_Pro_Parse_Broadcast_Data(header);
        }
        else if(DJI_Pro_Get_CmdSet_Id(header) == MY_BROADCAST_CMD_SET
                && DJI_Pro_Get_CmdCode_Id(header) == API_TRANSPARENT_DATA_TO_OBOARD)
        {
            if(p_user_rec_func)
            {
                len = (header->length - EXC_DATA_SIZE -2) > 100 ? 100 :
                          (header->length - EXC_DATA_SIZE -2);
                memcpy(buf,(unsigned char*)&header->magic + 2,len);
                p_user_rec_func(buf,len);
            }
        }
        else
        {
            if(p_user_handler_func)
            {
                p_user_handler_func(header);
            }
        }
        break;
    case 1:
    case 2:
        if(p_user_handler_func)
        {
            p_user_handler_func(header);
        }
        else
        {
            ProAckParameter param;
            printf("%s:Recv request,session id=%d,seq_num=%d\n",
                    __func__,header->session_id,header->sequence_number);
            if(header->session_id > 0)
            {
                buf[0] = buf[1] = 0;
                param.session_id = header->session_id;
                param.seq_num = header->sequence_number;
                param.need_encrypt = header->enc_type;
                param.buf = buf;
                param.length = 2;
                Pro_Ack_Interface(&param);
            }
        }
        break;
    }
}

int DJI_Pro_Register_Transparent_Transmission_Callback(Transparent_Transmission_Func user_rec_handler_entrance)
{
    p_user_rec_func = user_rec_handler_entrance;
    return 0;
}

int DJI_Pro_Setup(User_Handler_Func user_cmd_handler_entrance)
{
    Pro_Link_Setup();
    Pro_App_Recv_Set_Hook(DJI_Pro_App_Recv_Req_Data);
    p_user_handler_func = user_cmd_handler_entrance ? user_cmd_handler_entrance : 0;
    return 0;
}
