// MESSAGE ADSB_TRANSPONDER_STATIC_INPUT PACKING

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT 201

MAVPACKED(
typedef struct __mavlink_adsb_transponder_static_input_t {
 uint32_t ICAO; /*<  Vehicle address (24 bit). */
 uint16_t stallSpeed; /*<  Aircraft stall speed in cm/s. */
 char callsign[9]; /*<  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only). */
 uint8_t emitter; /*<  Transmitting vehicle type. See ADSB_EMITTER_CATEGORY enum. */
 uint8_t alwEncode; /*<  Aircraft length and width encoding (table 2-35 of DO-282B). */
 uint8_t gpsLatOffs; /*<  GPS antenna lateral offset (table 2-36 of DO-282B). */
 uint8_t gpsLonOffs; /*<  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B). */
}) mavlink_adsb_transponder_static_input_t;

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN 19
#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN 19
#define MAVLINK_MSG_ID_201_LEN 19
#define MAVLINK_MSG_ID_201_MIN_LEN 19

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC 126
#define MAVLINK_MSG_ID_201_CRC 126

#define MAVLINK_MSG_ADSB_TRANSPONDER_STATIC_INPUT_FIELD_CALLSIGN_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADSB_TRANSPONDER_STATIC_INPUT { \
	201, \
	"ADSB_TRANSPONDER_STATIC_INPUT", \
	7, \
	{  { "ICAO", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adsb_transponder_static_input_t, ICAO) }, \
         { "stallSpeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_adsb_transponder_static_input_t, stallSpeed) }, \
         { "callsign", NULL, MAVLINK_TYPE_CHAR, 9, 6, offsetof(mavlink_adsb_transponder_static_input_t, callsign) }, \
         { "emitter", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_adsb_transponder_static_input_t, emitter) }, \
         { "alwEncode", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_adsb_transponder_static_input_t, alwEncode) }, \
         { "gpsLatOffs", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_adsb_transponder_static_input_t, gpsLatOffs) }, \
         { "gpsLonOffs", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_adsb_transponder_static_input_t, gpsLonOffs) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADSB_TRANSPONDER_STATIC_INPUT { \
	"ADSB_TRANSPONDER_STATIC_INPUT", \
	7, \
	{  { "ICAO", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adsb_transponder_static_input_t, ICAO) }, \
         { "stallSpeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_adsb_transponder_static_input_t, stallSpeed) }, \
         { "callsign", NULL, MAVLINK_TYPE_CHAR, 9, 6, offsetof(mavlink_adsb_transponder_static_input_t, callsign) }, \
         { "emitter", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_adsb_transponder_static_input_t, emitter) }, \
         { "alwEncode", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_adsb_transponder_static_input_t, alwEncode) }, \
         { "gpsLatOffs", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_adsb_transponder_static_input_t, gpsLatOffs) }, \
         { "gpsLonOffs", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_adsb_transponder_static_input_t, gpsLonOffs) }, \
         } \
}
#endif

/**
 * @brief Pack a adsb_transponder_static_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ICAO  Vehicle address (24 bit). 
 * @param callsign  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only). 
 * @param emitter  Transmitting vehicle type. See ADSB_EMITTER_CATEGORY enum. 
 * @param alwEncode  Aircraft length and width encoding (table 2-35 of DO-282B). 
 * @param gpsLatOffs  GPS antenna lateral offset (table 2-36 of DO-282B). 
 * @param gpsLonOffs  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B). 
 * @param stallSpeed  Aircraft stall speed in cm/s. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_transponder_static_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t ICAO, const char *callsign, uint8_t emitter, uint8_t alwEncode, uint8_t gpsLatOffs, uint8_t gpsLonOffs, uint16_t stallSpeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN];
	_mav_put_uint32_t(buf, 0, ICAO);
	_mav_put_uint16_t(buf, 4, stallSpeed);
	_mav_put_uint8_t(buf, 15, emitter);
	_mav_put_uint8_t(buf, 16, alwEncode);
	_mav_put_uint8_t(buf, 17, gpsLatOffs);
	_mav_put_uint8_t(buf, 18, gpsLonOffs);
	_mav_put_char_array(buf, 6, callsign, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN);
#else
	mavlink_adsb_transponder_static_input_t packet;
	packet.ICAO = ICAO;
	packet.stallSpeed = stallSpeed;
	packet.emitter = emitter;
	packet.alwEncode = alwEncode;
	packet.gpsLatOffs = gpsLatOffs;
	packet.gpsLonOffs = gpsLonOffs;
	mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC);
}

/**
 * @brief Pack a adsb_transponder_static_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ICAO  Vehicle address (24 bit). 
 * @param callsign  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only). 
 * @param emitter  Transmitting vehicle type. See ADSB_EMITTER_CATEGORY enum. 
 * @param alwEncode  Aircraft length and width encoding (table 2-35 of DO-282B). 
 * @param gpsLatOffs  GPS antenna lateral offset (table 2-36 of DO-282B). 
 * @param gpsLonOffs  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B). 
 * @param stallSpeed  Aircraft stall speed in cm/s. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_transponder_static_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t ICAO,const char *callsign,uint8_t emitter,uint8_t alwEncode,uint8_t gpsLatOffs,uint8_t gpsLonOffs,uint16_t stallSpeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN];
	_mav_put_uint32_t(buf, 0, ICAO);
	_mav_put_uint16_t(buf, 4, stallSpeed);
	_mav_put_uint8_t(buf, 15, emitter);
	_mav_put_uint8_t(buf, 16, alwEncode);
	_mav_put_uint8_t(buf, 17, gpsLatOffs);
	_mav_put_uint8_t(buf, 18, gpsLonOffs);
	_mav_put_char_array(buf, 6, callsign, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN);
#else
	mavlink_adsb_transponder_static_input_t packet;
	packet.ICAO = ICAO;
	packet.stallSpeed = stallSpeed;
	packet.emitter = emitter;
	packet.alwEncode = alwEncode;
	packet.gpsLatOffs = gpsLatOffs;
	packet.gpsLonOffs = gpsLonOffs;
	mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC);
}

/**
 * @brief Encode a adsb_transponder_static_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adsb_transponder_static_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_transponder_static_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adsb_transponder_static_input_t* adsb_transponder_static_input)
{
	return mavlink_msg_adsb_transponder_static_input_pack(system_id, component_id, msg, adsb_transponder_static_input->ICAO, adsb_transponder_static_input->callsign, adsb_transponder_static_input->emitter, adsb_transponder_static_input->alwEncode, adsb_transponder_static_input->gpsLatOffs, adsb_transponder_static_input->gpsLonOffs, adsb_transponder_static_input->stallSpeed);
}

/**
 * @brief Encode a adsb_transponder_static_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adsb_transponder_static_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_transponder_static_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adsb_transponder_static_input_t* adsb_transponder_static_input)
{
	return mavlink_msg_adsb_transponder_static_input_pack_chan(system_id, component_id, chan, msg, adsb_transponder_static_input->ICAO, adsb_transponder_static_input->callsign, adsb_transponder_static_input->emitter, adsb_transponder_static_input->alwEncode, adsb_transponder_static_input->gpsLatOffs, adsb_transponder_static_input->gpsLonOffs, adsb_transponder_static_input->stallSpeed);
}

/**
 * @brief Send a adsb_transponder_static_input message
 * @param chan MAVLink channel to send the message
 *
 * @param ICAO  Vehicle address (24 bit). 
 * @param callsign  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only). 
 * @param emitter  Transmitting vehicle type. See ADSB_EMITTER_CATEGORY enum. 
 * @param alwEncode  Aircraft length and width encoding (table 2-35 of DO-282B). 
 * @param gpsLatOffs  GPS antenna lateral offset (table 2-36 of DO-282B). 
 * @param gpsLonOffs  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B). 
 * @param stallSpeed  Aircraft stall speed in cm/s. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adsb_transponder_static_input_send(mavlink_channel_t chan, uint32_t ICAO, const char *callsign, uint8_t emitter, uint8_t alwEncode, uint8_t gpsLatOffs, uint8_t gpsLonOffs, uint16_t stallSpeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN];
	_mav_put_uint32_t(buf, 0, ICAO);
	_mav_put_uint16_t(buf, 4, stallSpeed);
	_mav_put_uint8_t(buf, 15, emitter);
	_mav_put_uint8_t(buf, 16, alwEncode);
	_mav_put_uint8_t(buf, 17, gpsLatOffs);
	_mav_put_uint8_t(buf, 18, gpsLonOffs);
	_mav_put_char_array(buf, 6, callsign, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT, buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC);
#else
	mavlink_adsb_transponder_static_input_t packet;
	packet.ICAO = ICAO;
	packet.stallSpeed = stallSpeed;
	packet.emitter = emitter;
	packet.alwEncode = alwEncode;
	packet.gpsLatOffs = gpsLatOffs;
	packet.gpsLonOffs = gpsLonOffs;
	mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT, (const char *)&packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC);
#endif
}

/**
 * @brief Send a adsb_transponder_static_input message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adsb_transponder_static_input_send_struct(mavlink_channel_t chan, const mavlink_adsb_transponder_static_input_t* adsb_transponder_static_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adsb_transponder_static_input_send(chan, adsb_transponder_static_input->ICAO, adsb_transponder_static_input->callsign, adsb_transponder_static_input->emitter, adsb_transponder_static_input->alwEncode, adsb_transponder_static_input->gpsLatOffs, adsb_transponder_static_input->gpsLonOffs, adsb_transponder_static_input->stallSpeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT, (const char *)adsb_transponder_static_input, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adsb_transponder_static_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t ICAO, const char *callsign, uint8_t emitter, uint8_t alwEncode, uint8_t gpsLatOffs, uint8_t gpsLonOffs, uint16_t stallSpeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, ICAO);
	_mav_put_uint16_t(buf, 4, stallSpeed);
	_mav_put_uint8_t(buf, 15, emitter);
	_mav_put_uint8_t(buf, 16, alwEncode);
	_mav_put_uint8_t(buf, 17, gpsLatOffs);
	_mav_put_uint8_t(buf, 18, gpsLonOffs);
	_mav_put_char_array(buf, 6, callsign, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT, buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC);
#else
	mavlink_adsb_transponder_static_input_t *packet = (mavlink_adsb_transponder_static_input_t *)msgbuf;
	packet->ICAO = ICAO;
	packet->stallSpeed = stallSpeed;
	packet->emitter = emitter;
	packet->alwEncode = alwEncode;
	packet->gpsLatOffs = gpsLatOffs;
	packet->gpsLonOffs = gpsLonOffs;
	mav_array_memcpy(packet->callsign, callsign, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT, (const char *)packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE ADSB_TRANSPONDER_STATIC_INPUT UNPACKING


/**
 * @brief Get field ICAO from adsb_transponder_static_input message
 *
 * @return  Vehicle address (24 bit). 
 */
static inline uint32_t mavlink_msg_adsb_transponder_static_input_get_ICAO(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field callsign from adsb_transponder_static_input message
 *
 * @return  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only). 
 */
static inline uint16_t mavlink_msg_adsb_transponder_static_input_get_callsign(const mavlink_message_t* msg, char *callsign)
{
	return _MAV_RETURN_char_array(msg, callsign, 9,  6);
}

/**
 * @brief Get field emitter from adsb_transponder_static_input message
 *
 * @return  Transmitting vehicle type. See ADSB_EMITTER_CATEGORY enum. 
 */
static inline uint8_t mavlink_msg_adsb_transponder_static_input_get_emitter(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field alwEncode from adsb_transponder_static_input message
 *
 * @return  Aircraft length and width encoding (table 2-35 of DO-282B). 
 */
static inline uint8_t mavlink_msg_adsb_transponder_static_input_get_alwEncode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field gpsLatOffs from adsb_transponder_static_input message
 *
 * @return  GPS antenna lateral offset (table 2-36 of DO-282B). 
 */
static inline uint8_t mavlink_msg_adsb_transponder_static_input_get_gpsLatOffs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field gpsLonOffs from adsb_transponder_static_input message
 *
 * @return  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B). 
 */
static inline uint8_t mavlink_msg_adsb_transponder_static_input_get_gpsLonOffs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field stallSpeed from adsb_transponder_static_input message
 *
 * @return  Aircraft stall speed in cm/s. 
 */
static inline uint16_t mavlink_msg_adsb_transponder_static_input_get_stallSpeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a adsb_transponder_static_input message into a struct
 *
 * @param msg The message to decode
 * @param adsb_transponder_static_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_adsb_transponder_static_input_decode(const mavlink_message_t* msg, mavlink_adsb_transponder_static_input_t* adsb_transponder_static_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	adsb_transponder_static_input->ICAO = mavlink_msg_adsb_transponder_static_input_get_ICAO(msg);
	adsb_transponder_static_input->stallSpeed = mavlink_msg_adsb_transponder_static_input_get_stallSpeed(msg);
	mavlink_msg_adsb_transponder_static_input_get_callsign(msg, adsb_transponder_static_input->callsign);
	adsb_transponder_static_input->emitter = mavlink_msg_adsb_transponder_static_input_get_emitter(msg);
	adsb_transponder_static_input->alwEncode = mavlink_msg_adsb_transponder_static_input_get_alwEncode(msg);
	adsb_transponder_static_input->gpsLatOffs = mavlink_msg_adsb_transponder_static_input_get_gpsLatOffs(msg);
	adsb_transponder_static_input->gpsLonOffs = mavlink_msg_adsb_transponder_static_input_get_gpsLonOffs(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN? msg->len : MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN;
        memset(adsb_transponder_static_input, 0, MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_LEN);
	memcpy(adsb_transponder_static_input, _MAV_PAYLOAD(msg), len);
#endif
}
