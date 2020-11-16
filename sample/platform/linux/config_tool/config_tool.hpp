/*! @file config_tool.hpp
 *  @version 3.4.0
 *  @date September 5 2017
 *
 *  @brief
 *  Advanced control program allows for extra commands to
 *  to enable/disable USB connected flight, control power
 *  on the extension board and simulation.
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#ifndef CONFIG_TOOL_HPP
#define CONFIG_TOOL_HPP

#include "dji_vehicle.hpp"
#include "dji_log.hpp"
#include <iostream>
#include <dji_linux_helpers.hpp>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace DJI::OSDK;

class UtilityThread;

class ConfigTool
{
public:
  ConfigTool(Vehicle *vehicle);
  ~ConfigTool();

  bool doProcessPowerSupplyCtrl(std::string activeAction);
  bool doProcessUSBCtrl(std::string activeAction);
  bool doProcessSimulationCtrl(std::string GPSLatInDeg,
                               std::string GPSLonInDeg,
                               std::string activeAction);

#pragma pack(1)
  typedef struct USBCtrlData
  {
    uint16_t version;
    uint8_t  cmd;
  } USBCtrlData;

  typedef struct PowerSupplyData
  {
    uint8_t action;
    uint8_t cmd;
  } PowerSupplyData;

  typedef struct SimulationData
  {
    uint8_t	cmd;
    uint8_t	rc : 1;
    uint8_t	model : 1;
    uint8_t	resv : 6;
    uint8_t	freq;
    uint8_t	gps;
    double	lon;
    double	lat;
    double	height;
    uint8_t	roll : 1;
    uint8_t	pitch : 1;
    uint8_t	yaw : 1;
    uint8_t	x : 1;
    uint8_t	y : 1;
    uint8_t	z : 1;
    uint8_t	lati : 1;
    uint8_t	longti : 1;
    uint8_t	speed_x : 1;
    uint8_t	speed_y : 1;
    uint8_t	speed_z : 1;
    uint8_t	acc_x : 1;
    uint8_t	acc_y : 1;
    uint8_t	acc_z : 1;
    uint8_t	p : 1;
    uint8_t	q : 1;
    uint8_t	r : 1;
    uint8_t	rpm1 : 1;
    uint8_t	rpm2 : 1;
    uint8_t	rpm3 : 1;
    uint8_t	rpm4 : 1;
    uint8_t	rpm5 : 1;
    uint8_t	rpm6 : 1;
    uint8_t	rpm7 : 1;
    uint8_t	rpm8 : 1;
    uint8_t	duration : 1;
    uint8_t	led_color : 1;
    uint8_t	transform_state : 1;
    uint32_t	resv1 : 4;
    uint32_t	reserve;
  } SimulationData; 
#pragma pack()

  enum SIMULATION_ERROR_CODES
  {
    BEGIN_SIMULATED_FLIGHT_SUCCESS = 3,
    STOP_SIMULATED_FLIGHT_SUCCESS  = 5,
    BEGIN_SIMULATED_FLIGHT_ERROR   = 6,
    STOP_SIMULATED_FLIGHT_ERROR	   = 8
  };
 private:
  Vehicle *vehicle;
};

#endif /* CONFIG_TOOL_HPP */

