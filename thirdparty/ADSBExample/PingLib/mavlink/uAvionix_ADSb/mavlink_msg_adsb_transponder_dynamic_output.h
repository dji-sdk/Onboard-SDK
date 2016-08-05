// MESSAGE ADSB_TRANSPONDER_DYNAMIC_OUTPUT PACKING

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT 203

MAVPACKED(
typedef struct __mavlink_adsb_transponder_dynamic_output_t {
 uint8_t status; /*<  ADS-B transponder messages */
}) mavlink_adsb_transponder_dynamic_output_t;

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN 1
#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN 1
#define MAVLINK_MSG_ID_203_LEN 1
#define MAVLINK_MSG_ID_203_MIN_LEN 1

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC 85
#define MAVLINK_MSG_ID_203_CRC 85



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADSB_TRANSPONDER_DYNAMIC_OUTPUT { \
	203, \
	"ADSB_TRANSPONDER_DYNAMIC_OUTPUT", \
	1, \
	{  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_adsb_transponder_dynamic_output_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADSB_TRANSPONDER_DYNAMIC_OUTPUT { \
	"ADSB_TRANSPONDER_DYNAMIC_OUTPUT", \
	1, \
	{  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_adsb_transponder_dynamic_output_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a adsb_transponder_dynamic_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  ADS-B transponder messages 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN];
	_mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN);
#else
	mavlink_adsb_transponder_dynamic_output_t packet;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC);
}

/**
 * @brief Pack a adsb_transponder_dynamic_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  ADS-B transponder messages 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN];
	_mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN);
#else
	mavlink_adsb_transponder_dynamic_output_t packet;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC);
}

/**
 * @brief Encode a adsb_transponder_dynamic_output struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adsb_transponder_dynamic_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adsb_transponder_dynamic_output_t* adsb_transponder_dynamic_output)
{
	return mavlink_msg_adsb_transponder_dynamic_output_pack(system_id, component_id, msg, adsb_transponder_dynamic_output->status);
}

/**
 * @brief Encode a adsb_transponder_dynamic_output struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adsb_transponder_dynamic_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_output_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adsb_transponder_dynamic_output_t* adsb_transponder_dynamic_output)
{
	return mavlink_msg_adsb_transponder_dynamic_output_pack_chan(system_id, component_id, chan, msg, adsb_transponder_dynamic_output->status);
}

/**
 * @brief Send a adsb_transponder_dynamic_output message
 * @param chan MAVLink channel to send the message
 *
 * @param status  ADS-B transponder messages 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adsb_transponder_dynamic_output_send(mavlink_channel_t chan, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN];
	_mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT, buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC);
#else
	mavlink_adsb_transponder_dynamic_output_t packet;
	packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC);
#endif
}

/**
 * @brief Send a adsb_transponder_dynamic_output message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adsb_transponder_dynamic_output_send_struct(mavlink_channel_t chan, const mavlink_adsb_transponder_dynamic_output_t* adsb_transponder_dynamic_output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adsb_transponder_dynamic_output_send(chan, adsb_transponder_dynamic_output->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT, (const char *)adsb_transponder_dynamic_output, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adsb_transponder_dynamic_output_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT, buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC);
#else
	mavlink_adsb_transponder_dynamic_output_t *packet = (mavlink_adsb_transponder_dynamic_output_t *)msgbuf;
	packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE ADSB_TRANSPONDER_DYNAMIC_OUTPUT UNPACKING


/**
 * @brief Get field status from adsb_transponder_dynamic_output message
 *
 * @return  ADS-B transponder messages 
 */
static inline uint8_t mavlink_msg_adsb_transponder_dynamic_output_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a adsb_transponder_dynamic_output message into a struct
 *
 * @param msg The message to decode
 * @param adsb_transponder_dynamic_output C-struct to decode the message contents into
 */
static inline void mavlink_msg_adsb_transponder_dynamic_output_decode(const mavlink_message_t* msg, mavlink_adsb_transponder_dynamic_output_t* adsb_transponder_dynamic_output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	adsb_transponder_dynamic_output->status = mavlink_msg_adsb_transponder_dynamic_output_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN? msg->len : MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN;
        memset(adsb_transponder_dynamic_output, 0, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_LEN);
	memcpy(adsb_transponder_dynamic_output, _MAV_PAYLOAD(msg), len);
#endif
}
