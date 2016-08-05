/** @file
 *	@brief MAVLink comm protocol testsuite generated from uAvionix_ADSb.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef UAVIONIX_ADSB_TESTSUITE_H
#define UAVIONIX_ADSB_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_uAvionix_ADSb(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_uAvionix_ADSb(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_adsb_transponder_static_input(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_adsb_transponder_static_input_t packet_in = {
		963497464,17443,"GHIJKLMN",242,53,120,187
    };
	mavlink_adsb_transponder_static_input_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ICAO = packet_in.ICAO;
        packet1.stallSpeed = packet_in.stallSpeed;
        packet1.emitter = packet_in.emitter;
        packet1.alwEncode = packet_in.alwEncode;
        packet1.gpsLatOffs = packet_in.gpsLatOffs;
        packet1.gpsLonOffs = packet_in.gpsLonOffs;
        
        mav_array_memcpy(packet1.callsign, packet_in.callsign, sizeof(char)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADSB_TRANSPONDER_STATIC_INPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_static_input_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_adsb_transponder_static_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_static_input_pack(system_id, component_id, &msg , packet1.ICAO , packet1.callsign , packet1.emitter , packet1.alwEncode , packet1.gpsLatOffs , packet1.gpsLonOffs , packet1.stallSpeed );
	mavlink_msg_adsb_transponder_static_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_static_input_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ICAO , packet1.callsign , packet1.emitter , packet1.alwEncode , packet1.gpsLatOffs , packet1.gpsLonOffs , packet1.stallSpeed );
	mavlink_msg_adsb_transponder_static_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_adsb_transponder_static_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_static_input_send(MAVLINK_COMM_1 , packet1.ICAO , packet1.callsign , packet1.emitter , packet1.alwEncode , packet1.gpsLatOffs , packet1.gpsLonOffs , packet1.stallSpeed );
	mavlink_msg_adsb_transponder_static_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_adsb_transponder_dynamic_input(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_adsb_transponder_dynamic_input_t packet_in = {
		963497464,963497672,963497880,963498088,963498296,963498504,18483,18587,18691,18795,18899,19003,19107,247,58,125,192
    };
	mavlink_adsb_transponder_dynamic_input_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.utcTime = packet_in.utcTime;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altPres = packet_in.altPres;
        packet1.altGNSS = packet_in.altGNSS;
        packet1.accHoriz = packet_in.accHoriz;
        packet1.accVert = packet_in.accVert;
        packet1.accVel = packet_in.accVel;
        packet1.velVert = packet_in.velVert;
        packet1.nsVog = packet_in.nsVog;
        packet1.ewVog = packet_in.ewVog;
        packet1.state = packet_in.state;
        packet1.squawk = packet_in.squawk;
        packet1.fixType = packet_in.fixType;
        packet1.numSats = packet_in.numSats;
        packet1.emStatus = packet_in.emStatus;
        packet1.control = packet_in.control;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_INPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_input_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_adsb_transponder_dynamic_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_input_pack(system_id, component_id, &msg , packet1.utcTime , packet1.latitude , packet1.longitude , packet1.altPres , packet1.altGNSS , packet1.accHoriz , packet1.accVert , packet1.accVel , packet1.velVert , packet1.nsVog , packet1.ewVog , packet1.fixType , packet1.numSats , packet1.emStatus , packet1.control , packet1.state , packet1.squawk );
	mavlink_msg_adsb_transponder_dynamic_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_input_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.utcTime , packet1.latitude , packet1.longitude , packet1.altPres , packet1.altGNSS , packet1.accHoriz , packet1.accVert , packet1.accVel , packet1.velVert , packet1.nsVog , packet1.ewVog , packet1.fixType , packet1.numSats , packet1.emStatus , packet1.control , packet1.state , packet1.squawk );
	mavlink_msg_adsb_transponder_dynamic_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_adsb_transponder_dynamic_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_input_send(MAVLINK_COMM_1 , packet1.utcTime , packet1.latitude , packet1.longitude , packet1.altPres , packet1.altGNSS , packet1.accHoriz , packet1.accVert , packet1.accVel , packet1.velVert , packet1.nsVog , packet1.ewVog , packet1.fixType , packet1.numSats , packet1.emStatus , packet1.control , packet1.state , packet1.squawk );
	mavlink_msg_adsb_transponder_dynamic_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_adsb_transponder_dynamic_output(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_adsb_transponder_dynamic_output_t packet_in = {
		5
    };
	mavlink_adsb_transponder_dynamic_output_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADSB_TRANSPONDER_DYNAMIC_OUTPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_output_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_adsb_transponder_dynamic_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_output_pack(system_id, component_id, &msg , packet1.status );
	mavlink_msg_adsb_transponder_dynamic_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_output_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status );
	mavlink_msg_adsb_transponder_dynamic_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_adsb_transponder_dynamic_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adsb_transponder_dynamic_output_send(MAVLINK_COMM_1 , packet1.status );
	mavlink_msg_adsb_transponder_dynamic_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uAvionix_ADSb(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_adsb_transponder_static_input(system_id, component_id, last_msg);
	mavlink_test_adsb_transponder_dynamic_input(system_id, component_id, last_msg);
	mavlink_test_adsb_transponder_dynamic_output(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UAVIONIX_ADSB_TESTSUITE_H
