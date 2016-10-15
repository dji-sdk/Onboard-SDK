/*! @file Ping.cpp
 *
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Decode mavlink message from pingRX and put the traffic in a list.
 *
 *  Copyright 2016 DJI. All right reserved.
 */
#include <stm32f4xx.h>
#include "Ping.h"
#include <math.h>

using namespace DJI::onboardSDK;

mavlink_message_t Ping::mlMsg[MAVLINK_RX_BUFF_SIZE];  //holder for incomming msgs
uint8_t Ping::mLMsgPending;     //flag to indicate a incoming message from ping
uint8_t Ping::currentMLMsg;

Ping::Ping(CoreAPI *coreApi)
{
  mLMsgPending = 0;
  currentMLMsg = 0;
  api = coreApi;
  currentAvailPingTrafficIdx = -1;

  memset (PingTrafficList, 0, sizeof(PingTrafficList));

  for(uint32_t i = 0; i < NUM_PING_TRAFFIC_LIST; i++)
  {
    PingTrafficList[i].age = PING_TRAFFIC_VALID_TIMEOUT;
  }

  Uart4Config();
}

void Ping::parseOneByte( uint8_t data)
{
  uint8_t b;
  mavlink_status_t  status;
  b = mavlink_parse_char(MAVLINK_COMM_0, data, &mlMsg[mLMsgPending], &status);
  if (b)
  {
    mLMsgPending += b;
    mLMsgPending %= MAVLINK_RX_BUFF_SIZE;
  }
}


void Ping::processPing(void)
{
  while(currentMLMsg != mLMsgPending)
  {
    mavlink_message_t rxMsg = mlMsg[currentMLMsg];

    switch(rxMsg.msgid)
    {
      case MAVLINK_MSG_ID_ADSB_VEHICLE:
      {
        mavlink_adsb_vehicle_t data;
        mavlink_msg_adsb_vehicle_decode(&rxMsg, &data);

        PingTraffic * trafficPtr = getPingTrafficStructPtr(data.ICAO_address);

        if (trafficPtr == NULL)
          trafficPtr = addNewPingTraffic(data.ICAO_address);

        if (trafficPtr != NULL)
        {
          PositionData myPos = api->getBroadcastData().pos;

          trafficPtr->reportedToMobile = -1;

          if (data.flags & ADSB_FLAGS_VALID_COORDS)
          {
            trafficPtr->lat = data.lat;
            trafficPtr->lon = data.lon;
            trafficPtr->distanceToTraffic = distanceBetweenPoints(deg2rad((double)data.lat / 10000000), deg2rad((double)data.lon / 10000000), myPos.latitude, myPos.longitude);
          }

          if (data.flags & ADSB_FLAGS_VALID_ALTITUDE)
          {
            trafficPtr->altitude = data.altitude;
            trafficPtr->altitude_type = data.altitude_type;
          }

          if ((data.flags & ADSB_FLAGS_VALID_COORDS) &&
              (data.flags & ADSB_FLAGS_VALID_ALTITUDE))
          {
            trafficPtr->reportedToMobile = 0;
          }

          if (data.flags & ADSB_FLAGS_VALID_CALLSIGN)
          {
            memcpy(trafficPtr->callsign, data.callsign, NUM_CALLSIGN_BYTES);
          }

          if (data.flags & ADSB_FLAGS_VALID_HEADING)
          {
            trafficPtr->heading = data.heading;
          }

          if (data.flags & ADSB_FLAGS_VALID_VELOCITY)
          {
            trafficPtr->hor_velocity = data.hor_velocity * 10;
            trafficPtr->ver_velocity = data.ver_velocity * 10;
          }
          if (data.flags & ADSB_FLAGS_VALID_SQUAWK)
          {
            trafficPtr->squawk = data.squawk;
          }

          trafficPtr->emitter_type = data.emitter_type;
          trafficPtr->flags = data.flags;
          trafficPtr->tslc = data.tslc;

          if (data.flags & (ADSB_FLAGS_VALID_VELOCITY | ADSB_FLAGS_VALID_HEADING))
          {
            trafficPtr->nVel = (int32_t)(cosf(deg2rad(trafficPtr->heading / 100)) * trafficPtr->hor_velocity);
            trafficPtr->eVel = (int32_t)(sinf(deg2rad(trafficPtr->heading / 100)) * trafficPtr->hor_velocity);
          }

          if (data.flags & (ADSB_FLAGS_VALID_COORDS))
          {
            trafficPtr->courseToTraffic = courseToPoint(myPos.latitude, myPos.longitude, deg2rad((double)data.lat / 10000000), deg2rad((double)data.lon / 10000000));
          }

          trafficPtr->age = 0;
        }
      }
      break;
    }
    currentMLMsg ++;
    currentMLMsg %= MAVLINK_RX_BUFF_SIZE;
  }
}

void Ping::updateTrafficAge(void)
{
  uint8_t i;
  for(i = 0; i < NUM_PING_TRAFFIC_LIST; i++)
  {
    if (PingTrafficList[i].age < PING_TRAFFIC_VALID_TIMEOUT)
      PingTrafficList[i].age ++;
  }
}

uint8_t Ping::getNextAvailPingTraffic(PingTraffic &traffic)
{
  int32_t next = getNextPingTrafficIdx(currentAvailPingTrafficIdx);
  currentAvailPingTrafficIdx = next;
  if (currentAvailPingTrafficIdx == -1)
    return 0;
  else
  {
    traffic = PingTrafficList[currentAvailPingTrafficIdx];
    PingTrafficList[currentAvailPingTrafficIdx].reportedToMobile = 1;
    return 1;
  }
}

PingTraffic * Ping::getPingTrafficStructPtr(uint32_t ICAO)
{
  uint8_t i;
  PingTraffic * element = NULL;

  for(i = 0; i < NUM_PING_TRAFFIC_LIST; i++)
  {
    if (PingTrafficList[i].icao == ICAO)
    {
      element = &PingTrafficList[i];
      break;
    }
  }
  return element;
}

int32_t Ping::getNextPingTrafficIdx(int32_t current)
{
  int32_t next = current;
  for (int32_t i=1; i<NUM_PING_TRAFFIC_LIST-1; i++)
  {
    next += 1;
    next = (next >= NUM_PING_TRAFFIC_LIST) ? 0 : next;
    if ( (PingTrafficList[next].icao != 0) &&
         (PingTrafficList[next].age < PING_TRAFFIC_VALID_TIMEOUT) &&
         (PingTrafficList[next].reportedToMobile == 0))
    {
      return next;
    }
  }
  return -1;
}

PingTraffic * Ping::getNextPingTrafficPtr(PingTraffic * current)
{
  uint8_t i = 0;
  //uint8_t onceThrough = 0;
  uint8_t done = 0;
  uint8_t foundCurrent = (current == NULL);
  PingTraffic * next = NULL;

  while(!done)
  {
    if ( foundCurrent &&
         (PingTrafficList[i].age < PING_TRAFFIC_VALID_TIMEOUT) &&
         (PingTrafficList[i].reportedToMobile == 0) )
    {
      next = &PingTrafficList[i];
      done++;
    }
    if (!foundCurrent && &PingTrafficList[i] == current)
      foundCurrent++;

    if (++i >= NUM_PING_TRAFFIC_LIST)
    {
      done++;
//      if (onceThrough == 0)
//      {
//        i = 0;
//        onceThrough++;
//      }
//      else
//      {
//        done++;
//      }
    }
  }
  return next;
}

PingTraffic * Ping::addNewPingTraffic(uint32_t ICAO)
{
  uint8_t i;
  PingTraffic * element = NULL;

  for(i = 0; i < NUM_PING_TRAFFIC_LIST; i++)
  {
    if (PingTrafficList[i].age >= PING_TRAFFIC_VALID_TIMEOUT)
    {
      memset(&PingTrafficList[i], 0, PING_TRAFFIC_SIZE);
      PingTrafficList[i].icao = ICAO;
      //pingTraffic[i].flags = 0; //no need to do this
      element = &PingTrafficList[i];
      break;
    }
  }
  return element;
}


//Calc the dist between two coord's using the Spherical Law of Cosines
uint32_t distanceBetweenPoints(float lat1, float lon1, float lat2, float lon2)
{
    double latDiff = pow(sin((lat1 - lat2) / 2), 2);
    double lonDiff = cos(lat1) * cos(lat2) * pow(sin((lon1 - lon2) / 2), 2);
    double arcRad = 2 * asin(sqrt(latDiff + lonDiff));
    return((uint32_t)(arcRad * 6371008));
}

uint16_t courseToPoint(double lat1, double lon1, double lat2, double lon2)
{

  double x = sin(lon2-lon1)*cos(lat2);
  double y = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1);
  float tc1;
  if(y > 0)//heading north
  {
    if (x > 0) tc1 = rad2deg(atan(x/y));
    if (x < 0) tc1 = 270 + rad2deg(atan(y/-x));
    if (x == 0) tc1 = 0;
  }
  if (y < 0) //heading south
  {
    if (x > 0) tc1 = 90 + rad2deg(atan(-y/x));
    if (x < 0) tc1 = 270 - rad2deg(atan(y/x));
    if (x == 0) tc1 = 180;
  }
  if (y == 0)
  {
    if (x > 0) tc1 = 90;
    if (x < 0) tc1 = 270;
    if (x == 0) tc1 = 0;//[the 2 points are the same]
  }

  return((uint16_t)(tc1 * 100));
}

int8_t mod8(int8_t a, int8_t b)
{
  int8_t r = a % b;
  return r < 0 ? (r + b) : r;
}


void UART4_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);     //tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);     //rx

}

void UART4_Config(void)
{
  UART4_Gpio_Config();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 57600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(UART4, &USART_InitStructure);
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

  USART_Cmd(UART4, ENABLE);

  while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) != SET)
    ;
}

void USART4NVIC_Config()
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
}

void Uart4Config()
{
  UART4_Config();
  USART4NVIC_Config();
}


#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus
void UART4_IRQHandler(void)
{
  if (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == SET)
  {
    Ping::parseOneByte(USART_ReceiveData(UART4));
  }
}
#ifdef __cplusplus
}
#endif
