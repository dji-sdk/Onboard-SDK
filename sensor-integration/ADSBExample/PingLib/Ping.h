/*! @file Ping.h
 *
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Decode mavlink message from pingRX and put the traffic in a list.
 *
 *  Copyright 2016 DJI. All right reserved.
 */

#ifndef STM32PINGEXAMPLE_SRC_PING_H_
#define STM32PINGEXAMPLE_SRC_PING_H_

#include "DJI_API.h"
#include "mavlink/uAvionix_ADSb/mavlink.h"


#pragma pack(1)
typedef struct
{
  uint32_t icao;                //ICAO address
  int32_t  lat;                 //Latitude, expressed as degrees * 1E7
  int32_t  lon;                 //Longitude, expressed as degrees * 1E7
  int32_t  alt;                 //Altitude(ASL) in millimeters
  uint32_t velHoriz;            //The horizontal velocity in millimeters/second
  int32_t  velUp;               //The vertical velocity in millimeters/second, positive is up
  uint32_t distanceToMe;        //in Meters
  uint16_t heading;             //Course over ground in degree's * 100
  uint8_t  age;                 //Time in 0.1s since last update from ping
} MessageToMobile;
#pragma pack()

typedef struct
{
  uint32_t icao;                //ICAO address
  int32_t  lat;                 //Latitude, expressed as degrees * 1E7
  int32_t  lon;                 //Longitude, expressed as degrees * 1E7
  int32_t  altitude;            //Altitude(ASL) in millimeters
  uint32_t hor_velocity;        //The horizontal velocity in millimeters/second
  int32_t  ver_velocity;        //The vertical velocity in millimeters/second, positive is up
  int32_t  nVel;                //The north component of the velocity in millimeters/second
  int32_t  eVel;                //The eastern component of the velocity in millimeters/second
  uint32_t distanceToTraffic;   //in Meters
  uint16_t courseToTraffic;     //course to Traffic in degree's * 100
  uint16_t heading;             //Course over ground in degree's * 100
  uint16_t flags;               //Flags to indicate various statuses including valid data fields
  uint16_t squawk;              //Squawk code
  uint8_t  altitude_type;       //Type from ADSB_ALTITUDE_TYPE enum
  char     callsign[9];         //The callsign, 8+null
  uint8_t  emitter_type;        //Type from ADSB_EMITTER_TYPE enum
  uint8_t  tslc;                //Time since last communication in seconds
  uint8_t  age;                 //Time since last update from ping
  int8_t   reportedToMobile;    //Whether we have reported this message to mobile
} PingTraffic;


#define PING_TRAFFIC_SIZE    (sizeof(PingTraffic))
#define NUM_PING_TRAFFIC_LIST         100
#define PING_TRAFFIC_VALID_TIMEOUT    2
#define NUM_CALLSIGN_BYTES            8

#define M_PI                        3.14159265359
#define deg2rad(x)                  (((x) * M_PI) / 180.f)
#define rad2deg(x)                  (((x) * 180.f) / M_PI)

#define MAVLINK_RX_BUFF_SIZE          5


namespace DJI
{
  namespace onboardSDK
  {

    class Ping
    {
      public:
        Ping(CoreAPI *ControlAPI = 0);
        void processPing(void);
        void updateTrafficAge(void);

        uint8_t getNextAvailPingTraffic(PingTraffic &traffic);

        static void parseOneByte( uint8_t data);

      private:
        CoreAPI *api;
        PingTraffic PingTrafficList[NUM_PING_TRAFFIC_LIST];

        PingTraffic *addNewPingTraffic(uint32_t ICAO);
        PingTraffic *getNextPingTrafficPtr(PingTraffic * current);
        int32_t      getNextPingTrafficIdx(int32_t current);
        PingTraffic *getPingTrafficStructPtr(uint32_t ICAO);

        static mavlink_message_t mlMsg[MAVLINK_RX_BUFF_SIZE];  //holder for incomming msgs

        static uint8_t mLMsgPending;     //flag to indicate a incoming message from ping
        static uint8_t currentMLMsg;

        int32_t currentAvailPingTrafficIdx;

    };

  } //namespace OnboardSDK
} //namespace DJI


uint32_t distanceBetweenPoints(float lat1, float lon1, float lat2, float lon2);
uint16_t courseToPoint(double lat1, double lon1, double lat2, double lon2);
int8_t mod8(int8_t a, int8_t b);

void Uart4Config();

#endif /* STM32PINGEXAMPLE_SRC_PING_H_ */
