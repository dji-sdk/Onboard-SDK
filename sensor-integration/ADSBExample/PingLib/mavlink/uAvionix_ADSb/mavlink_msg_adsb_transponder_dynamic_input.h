// MESSAGE ADSB_TRANSPONDER_DYNAMIC_INPUT PACKING

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT 202

MAVPACKED(
typedef struct __mavlink_adsb_transponder_dynamic_input_t {
 uint32_t utcTime; /*<  UTC time in s since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX. */
 int32_t latitude; /*<  Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. */
 int32_t longitude; /*<  Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. */
 int32_t altPres; /*<  Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (meters * 1E3). (up +ve). If unknown set to INT32_MAX. */
 int32_t altGNSS; /*<  Altitude (meters * 1E3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX. */
 uint32_t accHoriz; /*<  Horizontal accuracy (mm). If unknown set to UINT32_MAX. */
 uint16_t accVert; /*<  Vertical accuracy (cm). If unknown set to UINT16_MAX. */
 uint16_t accVel; /*<  Velocity accuracy (m/s * 1E3). If unknown set to UINT16_MAX. */
 int16_t velVert; /*<  GPS vertical speed (m/s * 1E2). If unknown set to INT16_MAX. */
 int16_t nsVog; /*<  North-South velocity over ground (m/s * 10) North +ve. If unknown set to INT16_MAX. */
 int16_t ewVog; /*<  East-West velocity over ground (m/s * 10) East +ve. If unknown set to INT16_MAX. */
 uint16_t state; /*<  ADS-B transponder dynamic input state flags. */
 uint16_t squawk; /*<  Mode A code (typically 1200 [0x04B0] for VFR). */
 uint8_t fixType; /*<  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. */
 uint8_t numSats; /*<  Number of satellites visible. If unknown set to UINT8_MAX. */
 uint8_t emStatus; /*<  Emergency status (table 2-78 of DO-260B). */
 uint8_t control; /*<  ADS-B transponder dynamic input control flags. */
}) mavlink_adsb_transponder_dynamic_input_t;

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN 42
#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN 42
#define MAVLINK_MSG_ID_202_LEN 42
#define MAVLINK_MSG_ID_202_MIN_LEN 42

#define MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC 7
#define MAVLINK_MSG_ID_202_CRC 7



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADSB_TRANSPONDER_DYNAMIC_INPUT { \
	202, \
	"ADSB_TRANSPONDER_DYNAMIC_INPUT", \
	17, \
	{  { "utcTime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adsb_transponder_dynamic_input_t, utcTime) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_adsb_transponder_dynamic_input_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_adsb_transponder_dynamic_input_t, longitude) }, \
         { "altPres", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_adsb_transponder_dynamic_input_t, altPres) }, \
         { "altGNSS", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_adsb_transponder_dynamic_input_t, altGNSS) }, \
         { "accHoriz", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_adsb_transponder_dynamic_input_t, accHoriz) }, \
         { "accVert", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_adsb_transponder_dynamic_input_t, accVert) }, \
         { "accVel", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_adsb_transponder_dynamic_input_t, accVel) }, \
         { "velVert", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_adsb_transponder_dynamic_input_t, velVert) }, \
         { "nsVog", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_adsb_transponder_dynamic_input_t, nsVog) }, \
         { "ewVog", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_adsb_transponder_dynamic_input_t, ewVog) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_adsb_transponder_dynamic_input_t, state) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_adsb_transponder_dynamic_input_t, squawk) }, \
         { "fixType", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_adsb_transponder_dynamic_input_t, fixType) }, \
         { "numSats", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_adsb_transponder_dynamic_input_t, numSats) }, \
         { "emStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_adsb_transponder_dynamic_input_t, emStatus) }, \
         { "control", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_adsb_transponder_dynamic_input_t, control) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADSB_TRANSPONDER_DYNAMIC_INPUT { \
	"ADSB_TRANSPONDER_DYNAMIC_INPUT", \
	17, \
	{  { "utcTime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adsb_transponder_dynamic_input_t, utcTime) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_adsb_transponder_dynamic_input_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_adsb_transponder_dynamic_input_t, longitude) }, \
         { "altPres", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_adsb_transponder_dynamic_input_t, altPres) }, \
         { "altGNSS", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_adsb_transponder_dynamic_input_t, altGNSS) }, \
         { "accHoriz", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_adsb_transponder_dynamic_input_t, accHoriz) }, \
         { "accVert", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_adsb_transponder_dynamic_input_t, accVert) }, \
         { "accVel", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_adsb_transponder_dynamic_input_t, accVel) }, \
         { "velVert", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_adsb_transponder_dynamic_input_t, velVert) }, \
         { "nsVog", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_adsb_transponder_dynamic_input_t, nsVog) }, \
         { "ewVog", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_adsb_transponder_dynamic_input_t, ewVog) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_adsb_transponder_dynamic_input_t, state) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_adsb_transponder_dynamic_input_t, squawk) }, \
         { "fixType", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_adsb_transponder_dynamic_input_t, fixType) }, \
         { "numSats", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_adsb_transponder_dynamic_input_t, numSats) }, \
         { "emStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_adsb_transponder_dynamic_input_t, emStatus) }, \
         { "control", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_adsb_transponder_dynamic_input_t, control) }, \
         } \
}
#endif

/**
 * @brief Pack a adsb_transponder_dynamic_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utcTime  UTC time in s since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX. 
 * @param latitude  Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 * @param longitude  Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 * @param altPres  Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (meters * 1E3). (up +ve). If unknown set to INT32_MAX. 
 * @param altGNSS  Altitude (meters * 1E3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX. 
 * @param accHoriz  Horizontal accuracy (mm). If unknown set to UINT32_MAX. 
 * @param accVert  Vertical accuracy (cm). If unknown set to UINT16_MAX. 
 * @param accVel  Velocity accuracy (m/s * 1E3). If unknown set to UINT16_MAX. 
 * @param velVert  GPS vertical speed (m/s * 1E2). If unknown set to INT16_MAX. 
 * @param nsVog  North-South velocity over ground (m/s * 10) North +ve. If unknown set to INT16_MAX. 
 * @param ewVog  East-West velocity over ground (m/s * 10) East +ve. If unknown set to INT16_MAX. 
 * @param fixType  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. 
 * @param numSats  Number of satellites visible. If unknown set to UINT8_MAX. 
 * @param emStatus  Emergency status (table 2-78 of DO-260B). 
 * @param control  ADS-B transponder dynamic input control flags. 
 * @param state  ADS-B transponder dynamic input state flags. 
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR). 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t utcTime, int32_t latitude, int32_t longitude, int32_t altPres, int32_t altGNSS, uint32_t accHoriz, uint16_t accVert, uint16_t accVel, int16_t velVert, int16_t nsVog, int16_t ewVog, uint8_t fixType, uint8_t numSats, uint8_t emStatus, uint8_t control, uint16_t state, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN];
	_mav_put_uint32_t(buf, 0, utcTime);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, altPres);
	_mav_put_int32_t(buf, 16, altGNSS);
	_mav_put_uint32_t(buf, 20, accHoriz);
	_mav_put_uint16_t(buf, 24, accVert);
	_mav_put_uint16_t(buf, 26, accVel);
	_mav_put_int16_t(buf, 28, velVert);
	_mav_put_int16_t(buf, 30, nsVog);
	_mav_put_int16_t(buf, 32, ewVog);
	_mav_put_uint16_t(buf, 34, state);
	_mav_put_uint16_t(buf, 36, squawk);
	_mav_put_uint8_t(buf, 38, fixType);
	_mav_put_uint8_t(buf, 39, numSats);
	_mav_put_uint8_t(buf, 40, emStatus);
	_mav_put_uint8_t(buf, 41, control);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN);
#else
	mavlink_adsb_transponder_dynamic_input_t packet;
	packet.utcTime = utcTime;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altPres = altPres;
	packet.altGNSS = altGNSS;
	packet.accHoriz = accHoriz;
	packet.accVert = accVert;
	packet.accVel = accVel;
	packet.velVert = velVert;
	packet.nsVog = nsVog;
	packet.ewVog = ewVog;
	packet.state = state;
	packet.squawk = squawk;
	packet.fixType = fixType;
	packet.numSats = numSats;
	packet.emStatus = emStatus;
	packet.control = control;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC);
}

/**
 * @brief Pack a adsb_transponder_dynamic_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param utcTime  UTC time in s since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX. 
 * @param latitude  Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 * @param longitude  Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 * @param altPres  Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (meters * 1E3). (up +ve). If unknown set to INT32_MAX. 
 * @param altGNSS  Altitude (meters * 1E3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX. 
 * @param accHoriz  Horizontal accuracy (mm). If unknown set to UINT32_MAX. 
 * @param accVert  Vertical accuracy (cm). If unknown set to UINT16_MAX. 
 * @param accVel  Velocity accuracy (m/s * 1E3). If unknown set to UINT16_MAX. 
 * @param velVert  GPS vertical speed (m/s * 1E2). If unknown set to INT16_MAX. 
 * @param nsVog  North-South velocity over ground (m/s * 10) North +ve. If unknown set to INT16_MAX. 
 * @param ewVog  East-West velocity over ground (m/s * 10) East +ve. If unknown set to INT16_MAX. 
 * @param fixType  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. 
 * @param numSats  Number of satellites visible. If unknown set to UINT8_MAX. 
 * @param emStatus  Emergency status (table 2-78 of DO-260B). 
 * @param control  ADS-B transponder dynamic input control flags. 
 * @param state  ADS-B transponder dynamic input state flags. 
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR). 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t utcTime,int32_t latitude,int32_t longitude,int32_t altPres,int32_t altGNSS,uint32_t accHoriz,uint16_t accVert,uint16_t accVel,int16_t velVert,int16_t nsVog,int16_t ewVog,uint8_t fixType,uint8_t numSats,uint8_t emStatus,uint8_t control,uint16_t state,uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN];
	_mav_put_uint32_t(buf, 0, utcTime);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, altPres);
	_mav_put_int32_t(buf, 16, altGNSS);
	_mav_put_uint32_t(buf, 20, accHoriz);
	_mav_put_uint16_t(buf, 24, accVert);
	_mav_put_uint16_t(buf, 26, accVel);
	_mav_put_int16_t(buf, 28, velVert);
	_mav_put_int16_t(buf, 30, nsVog);
	_mav_put_int16_t(buf, 32, ewVog);
	_mav_put_uint16_t(buf, 34, state);
	_mav_put_uint16_t(buf, 36, squawk);
	_mav_put_uint8_t(buf, 38, fixType);
	_mav_put_uint8_t(buf, 39, numSats);
	_mav_put_uint8_t(buf, 40, emStatus);
	_mav_put_uint8_t(buf, 41, control);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN);
#else
	mavlink_adsb_transponder_dynamic_input_t packet;
	packet.utcTime = utcTime;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altPres = altPres;
	packet.altGNSS = altGNSS;
	packet.accHoriz = accHoriz;
	packet.accVert = accVert;
	packet.accVel = accVel;
	packet.velVert = velVert;
	packet.nsVog = nsVog;
	packet.ewVog = ewVog;
	packet.state = state;
	packet.squawk = squawk;
	packet.fixType = fixType;
	packet.numSats = numSats;
	packet.emStatus = emStatus;
	packet.control = control;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC);
}

/**
 * @brief Encode a adsb_transponder_dynamic_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adsb_transponder_dynamic_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adsb_transponder_dynamic_input_t* adsb_transponder_dynamic_input)
{
	return mavlink_msg_adsb_transponder_dynamic_input_pack(system_id, component_id, msg, adsb_transponder_dynamic_input->utcTime, adsb_transponder_dynamic_input->latitude, adsb_transponder_dynamic_input->longitude, adsb_transponder_dynamic_input->altPres, adsb_transponder_dynamic_input->altGNSS, adsb_transponder_dynamic_input->accHoriz, adsb_transponder_dynamic_input->accVert, adsb_transponder_dynamic_input->accVel, adsb_transponder_dynamic_input->velVert, adsb_transponder_dynamic_input->nsVog, adsb_transponder_dynamic_input->ewVog, adsb_transponder_dynamic_input->fixType, adsb_transponder_dynamic_input->numSats, adsb_transponder_dynamic_input->emStatus, adsb_transponder_dynamic_input->control, adsb_transponder_dynamic_input->state, adsb_transponder_dynamic_input->squawk);
}

/**
 * @brief Encode a adsb_transponder_dynamic_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adsb_transponder_dynamic_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adsb_transponder_dynamic_input_t* adsb_transponder_dynamic_input)
{
	return mavlink_msg_adsb_transponder_dynamic_input_pack_chan(system_id, component_id, chan, msg, adsb_transponder_dynamic_input->utcTime, adsb_transponder_dynamic_input->latitude, adsb_transponder_dynamic_input->longitude, adsb_transponder_dynamic_input->altPres, adsb_transponder_dynamic_input->altGNSS, adsb_transponder_dynamic_input->accHoriz, adsb_transponder_dynamic_input->accVert, adsb_transponder_dynamic_input->accVel, adsb_transponder_dynamic_input->velVert, adsb_transponder_dynamic_input->nsVog, adsb_transponder_dynamic_input->ewVog, adsb_transponder_dynamic_input->fixType, adsb_transponder_dynamic_input->numSats, adsb_transponder_dynamic_input->emStatus, adsb_transponder_dynamic_input->control, adsb_transponder_dynamic_input->state, adsb_transponder_dynamic_input->squawk);
}

/**
 * @brief Send a adsb_transponder_dynamic_input message
 * @param chan MAVLink channel to send the message
 *
 * @param utcTime  UTC time in s since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX. 
 * @param latitude  Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 * @param longitude  Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 * @param altPres  Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (meters * 1E3). (up +ve). If unknown set to INT32_MAX. 
 * @param altGNSS  Altitude (meters * 1E3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX. 
 * @param accHoriz  Horizontal accuracy (mm). If unknown set to UINT32_MAX. 
 * @param accVert  Vertical accuracy (cm). If unknown set to UINT16_MAX. 
 * @param accVel  Velocity accuracy (m/s * 1E3). If unknown set to UINT16_MAX. 
 * @param velVert  GPS vertical speed (m/s * 1E2). If unknown set to INT16_MAX. 
 * @param nsVog  North-South velocity over ground (m/s * 10) North +ve. If unknown set to INT16_MAX. 
 * @param ewVog  East-West velocity over ground (m/s * 10) East +ve. If unknown set to INT16_MAX. 
 * @param fixType  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. 
 * @param numSats  Number of satellites visible. If unknown set to UINT8_MAX. 
 * @param emStatus  Emergency status (table 2-78 of DO-260B). 
 * @param control  ADS-B transponder dynamic input control flags. 
 * @param state  ADS-B transponder dynamic input state flags. 
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR). 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adsb_transponder_dynamic_input_send(mavlink_channel_t chan, uint32_t utcTime, int32_t latitude, int32_t longitude, int32_t altPres, int32_t altGNSS, uint32_t accHoriz, uint16_t accVert, uint16_t accVel, int16_t velVert, int16_t nsVog, int16_t ewVog, uint8_t fixType, uint8_t numSats, uint8_t emStatus, uint8_t control, uint16_t state, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN];
	_mav_put_uint32_t(buf, 0, utcTime);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, altPres);
	_mav_put_int32_t(buf, 16, altGNSS);
	_mav_put_uint32_t(buf, 20, accHoriz);
	_mav_put_uint16_t(buf, 24, accVert);
	_mav_put_uint16_t(buf, 26, accVel);
	_mav_put_int16_t(buf, 28, velVert);
	_mav_put_int16_t(buf, 30, nsVog);
	_mav_put_int16_t(buf, 32, ewVog);
	_mav_put_uint16_t(buf, 34, state);
	_mav_put_uint16_t(buf, 36, squawk);
	_mav_put_uint8_t(buf, 38, fixType);
	_mav_put_uint8_t(buf, 39, numSats);
	_mav_put_uint8_t(buf, 40, emStatus);
	_mav_put_uint8_t(buf, 41, control);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT, buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC);
#else
	mavlink_adsb_transponder_dynamic_input_t packet;
	packet.utcTime = utcTime;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altPres = altPres;
	packet.altGNSS = altGNSS;
	packet.accHoriz = accHoriz;
	packet.accVert = accVert;
	packet.accVel = accVel;
	packet.velVert = velVert;
	packet.nsVog = nsVog;
	packet.ewVog = ewVog;
	packet.state = state;
	packet.squawk = squawk;
	packet.fixType = fixType;
	packet.numSats = numSats;
	packet.emStatus = emStatus;
	packet.control = control;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT, (const char *)&packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC);
#endif
}

/**
 * @brief Send a adsb_transponder_dynamic_input message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adsb_transponder_dynamic_input_send_struct(mavlink_channel_t chan, const mavlink_adsb_transponder_dynamic_input_t* adsb_transponder_dynamic_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adsb_transponder_dynamic_input_send(chan, adsb_transponder_dynamic_input->utcTime, adsb_transponder_dynamic_input->latitude, adsb_transponder_dynamic_input->longitude, adsb_transponder_dynamic_input->altPres, adsb_transponder_dynamic_input->altGNSS, adsb_transponder_dynamic_input->accHoriz, adsb_transponder_dynamic_input->accVert, adsb_transponder_dynamic_input->accVel, adsb_transponder_dynamic_input->velVert, adsb_transponder_dynamic_input->nsVog, adsb_transponder_dynamic_input->ewVog, adsb_transponder_dynamic_input->fixType, adsb_transponder_dynamic_input->numSats, adsb_transponder_dynamic_input->emStatus, adsb_transponder_dynamic_input->control, adsb_transponder_dynamic_input->state, adsb_transponder_dynamic_input->squawk);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT, (const char *)adsb_transponder_dynamic_input, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adsb_transponder_dynamic_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t utcTime, int32_t latitude, int32_t longitude, int32_t altPres, int32_t altGNSS, uint32_t accHoriz, uint16_t accVert, uint16_t accVel, int16_t velVert, int16_t nsVog, int16_t ewVog, uint8_t fixType, uint8_t numSats, uint8_t emStatus, uint8_t control, uint16_t state, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, utcTime);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, altPres);
	_mav_put_int32_t(buf, 16, altGNSS);
	_mav_put_uint32_t(buf, 20, accHoriz);
	_mav_put_uint16_t(buf, 24, accVert);
	_mav_put_uint16_t(buf, 26, accVel);
	_mav_put_int16_t(buf, 28, velVert);
	_mav_put_int16_t(buf, 30, nsVog);
	_mav_put_int16_t(buf, 32, ewVog);
	_mav_put_uint16_t(buf, 34, state);
	_mav_put_uint16_t(buf, 36, squawk);
	_mav_put_uint8_t(buf, 38, fixType);
	_mav_put_uint8_t(buf, 39, numSats);
	_mav_put_uint8_t(buf, 40, emStatus);
	_mav_put_uint8_t(buf, 41, control);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT, buf, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC);
#else
	mavlink_adsb_transponder_dynamic_input_t *packet = (mavlink_adsb_transponder_dynamic_input_t *)msgbuf;
	packet->utcTime = utcTime;
	packet->latitude = latitude;
	packet->longitude = longitude;
	packet->altPres = altPres;
	packet->altGNSS = altGNSS;
	packet->accHoriz = accHoriz;
	packet->accVert = accVert;
	packet->accVel = accVel;
	packet->velVert = velVert;
	packet->nsVog = nsVog;
	packet->ewVog = ewVog;
	packet->state = state;
	packet->squawk = squawk;
	packet->fixType = fixType;
	packet->numSats = numSats;
	packet->emStatus = emStatus;
	packet->control = control;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT, (const char *)packet, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE ADSB_TRANSPONDER_DYNAMIC_INPUT UNPACKING


/**
 * @brief Get field utcTime from adsb_transponder_dynamic_input message
 *
 * @return  UTC time in s since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX. 
 */
static inline uint32_t mavlink_msg_adsb_transponder_dynamic_input_get_utcTime(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field latitude from adsb_transponder_dynamic_input message
 *
 * @return  Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 */
static inline int32_t mavlink_msg_adsb_transponder_dynamic_input_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field longitude from adsb_transponder_dynamic_input message
 *
 * @return  Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX. 
 */
static inline int32_t mavlink_msg_adsb_transponder_dynamic_input_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altPres from adsb_transponder_dynamic_input message
 *
 * @return  Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (meters * 1E3). (up +ve). If unknown set to INT32_MAX. 
 */
static inline int32_t mavlink_msg_adsb_transponder_dynamic_input_get_altPres(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field altGNSS from adsb_transponder_dynamic_input message
 *
 * @return  Altitude (meters * 1E3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX. 
 */
static inline int32_t mavlink_msg_adsb_transponder_dynamic_input_get_altGNSS(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field accHoriz from adsb_transponder_dynamic_input message
 *
 * @return  Horizontal accuracy (mm). If unknown set to UINT32_MAX. 
 */
static inline uint32_t mavlink_msg_adsb_transponder_dynamic_input_get_accHoriz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field accVert from adsb_transponder_dynamic_input message
 *
 * @return  Vertical accuracy (cm). If unknown set to UINT16_MAX. 
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_get_accVert(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field accVel from adsb_transponder_dynamic_input message
 *
 * @return  Velocity accuracy (m/s * 1E3). If unknown set to UINT16_MAX. 
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_get_accVel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field velVert from adsb_transponder_dynamic_input message
 *
 * @return  GPS vertical speed (m/s * 1E2). If unknown set to INT16_MAX. 
 */
static inline int16_t mavlink_msg_adsb_transponder_dynamic_input_get_velVert(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field nsVog from adsb_transponder_dynamic_input message
 *
 * @return  North-South velocity over ground (m/s * 10) North +ve. If unknown set to INT16_MAX. 
 */
static inline int16_t mavlink_msg_adsb_transponder_dynamic_input_get_nsVog(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field ewVog from adsb_transponder_dynamic_input message
 *
 * @return  East-West velocity over ground (m/s * 10) East +ve. If unknown set to INT16_MAX. 
 */
static inline int16_t mavlink_msg_adsb_transponder_dynamic_input_get_ewVog(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field fixType from adsb_transponder_dynamic_input message
 *
 * @return  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. 
 */
static inline uint8_t mavlink_msg_adsb_transponder_dynamic_input_get_fixType(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field numSats from adsb_transponder_dynamic_input message
 *
 * @return  Number of satellites visible. If unknown set to UINT8_MAX. 
 */
static inline uint8_t mavlink_msg_adsb_transponder_dynamic_input_get_numSats(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field emStatus from adsb_transponder_dynamic_input message
 *
 * @return  Emergency status (table 2-78 of DO-260B). 
 */
static inline uint8_t mavlink_msg_adsb_transponder_dynamic_input_get_emStatus(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field control from adsb_transponder_dynamic_input message
 *
 * @return  ADS-B transponder dynamic input control flags. 
 */
static inline uint8_t mavlink_msg_adsb_transponder_dynamic_input_get_control(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field state from adsb_transponder_dynamic_input message
 *
 * @return  ADS-B transponder dynamic input state flags. 
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_get_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field squawk from adsb_transponder_dynamic_input message
 *
 * @return  Mode A code (typically 1200 [0x04B0] for VFR). 
 */
static inline uint16_t mavlink_msg_adsb_transponder_dynamic_input_get_squawk(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Decode a adsb_transponder_dynamic_input message into a struct
 *
 * @param msg The message to decode
 * @param adsb_transponder_dynamic_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_adsb_transponder_dynamic_input_decode(const mavlink_message_t* msg, mavlink_adsb_transponder_dynamic_input_t* adsb_transponder_dynamic_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	adsb_transponder_dynamic_input->utcTime = mavlink_msg_adsb_transponder_dynamic_input_get_utcTime(msg);
	adsb_transponder_dynamic_input->latitude = mavlink_msg_adsb_transponder_dynamic_input_get_latitude(msg);
	adsb_transponder_dynamic_input->longitude = mavlink_msg_adsb_transponder_dynamic_input_get_longitude(msg);
	adsb_transponder_dynamic_input->altPres = mavlink_msg_adsb_transponder_dynamic_input_get_altPres(msg);
	adsb_transponder_dynamic_input->altGNSS = mavlink_msg_adsb_transponder_dynamic_input_get_altGNSS(msg);
	adsb_transponder_dynamic_input->accHoriz = mavlink_msg_adsb_transponder_dynamic_input_get_accHoriz(msg);
	adsb_transponder_dynamic_input->accVert = mavlink_msg_adsb_transponder_dynamic_input_get_accVert(msg);
	adsb_transponder_dynamic_input->accVel = mavlink_msg_adsb_transponder_dynamic_input_get_accVel(msg);
	adsb_transponder_dynamic_input->velVert = mavlink_msg_adsb_transponder_dynamic_input_get_velVert(msg);
	adsb_transponder_dynamic_input->nsVog = mavlink_msg_adsb_transponder_dynamic_input_get_nsVog(msg);
	adsb_transponder_dynamic_input->ewVog = mavlink_msg_adsb_transponder_dynamic_input_get_ewVog(msg);
	adsb_transponder_dynamic_input->state = mavlink_msg_adsb_transponder_dynamic_input_get_state(msg);
	adsb_transponder_dynamic_input->squawk = mavlink_msg_adsb_transponder_dynamic_input_get_squawk(msg);
	adsb_transponder_dynamic_input->fixType = mavlink_msg_adsb_transponder_dynamic_input_get_fixType(msg);
	adsb_transponder_dynamic_input->numSats = mavlink_msg_adsb_transponder_dynamic_input_get_numSats(msg);
	adsb_transponder_dynamic_input->emStatus = mavlink_msg_adsb_transponder_dynamic_input_get_emStatus(msg);
	adsb_transponder_dynamic_input->control = mavlink_msg_adsb_transponder_dynamic_input_get_control(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN? msg->len : MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN;
        memset(adsb_transponder_dynamic_input, 0, MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_LEN);
	memcpy(adsb_transponder_dynamic_input, _MAV_PAYLOAD(msg), len);
#endif
}
