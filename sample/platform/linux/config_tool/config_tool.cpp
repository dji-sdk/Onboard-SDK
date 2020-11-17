/*! @file payloads/main_sync.cpp
 *  @version 4.0.0
 *  @date November 19 2020
 *
 *  @brief
 *  main for CameraManager usage in a Linux environment.
 *  Shows example usage of CameraManager synchronous APIs.
 *
 *  @Copyright (c) 2019 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "config_tool.hpp"
#include "dji_linker.hpp"

ConfigTool::ConfigTool(Vehicle *vehicle) {
  this->vehicle = vehicle;
}

ConfigTool::~ConfigTool() {
}

void show_usage(std::string name)
{
   std::cerr << "Usage: " << name.c_str() << " --usb-port <port_name> --config-file <file_name> <command> <action> <options>\n"
             << "\t--help\t\t\t\tDisplay program commands and options\n"
             << "\t--usb-port <port_name>\t\tUSB port supporting connection between M210/M300\n"
             << "\t\t\t\t\tAdvanced Features channel and compute platform\n"
             << "\t\t\t\t\tYou can set the port with \"acm_port : /dev/ttyACMx\" in your config-file.\n"
             << "\t\t\t\t\tThis setting will cover the \"acm_port\" from the config-file(if exist).\n"
             << "\t--config-file <file_name>\t\tUser configuration file\n"
             << "\t--power-supply on/off\t\tSet power supply on the M210 extension board\n"
             << "\t--usb-connected-flight on/off\tEnable/disable USB connected flight on M210.\n"
             << "\t\t\t\t\tAfter OSDK 4.0, connected USB flight will be enabled automatically on \n"
             << "\t\t\t\t\tinitialization for M210.\n"
             << "\t--simulation on/off\t\tEnable/disable M210/M300 Aircraft simulation\n"
             << "\tSimulation Options:\n"
             << "\t--latitude <latitude_value>\n" 
             << "\t--longitude <longitude_value>\n"
             << std::endl;
}

bool ConfigTool::doProcessUSBCtrl(std::string activeAction)
{
  bool ctrlCmd = false;
  if(activeAction.compare("on") == 0)
  {
    ctrlCmd = true;
  }
  else if(activeAction.compare("off") == 0)
  {
    ctrlCmd = false;
  }
  else
  {
    DERROR("Error: Unknown action given!\n");
    return false;
  }

  return vehicle->setUSBFlightOn(ctrlCmd);
}

bool ConfigTool::doProcessPowerSupplyCtrl(std::string activeAction)
{
  if(activeAction.empty())
  {
    DERROR("Error: action not set!\n");
    return false;
  }

  if (strncmp(vehicle->getHwVersion(), Version::M210V2, 5)
      && strncmp(vehicle->getHwVersion(), Version::M210, 5)) {
    DERROR("Power supply setting only for M210 series!\n");
    return false;
  }

  ConfigTool::PowerSupplyData data;
  memset(&data, 0, sizeof(data));

  // Action: set power supply
  data.action = 0;

  if(activeAction.compare("on") == 0)
  {
    data.cmd = 1;
  }
  else if(activeAction.compare("off") == 0)
  {
    data.cmd = 0;
  }
  else
  {
    DERROR("Error: Unknown action given!\n");
    return false;
  }

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t cbData[1024] = {0};

  cmdInfo.cmdSet = 0x19;
  cmdInfo.cmdId = 0x02;
  cmdInfo.dataLen = sizeof(data);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(0x04, 0x07);
  cmdInfo.sender = vehicle->linker->getLocalSenderId();
  DSTATUS("Trying to set power supply for M210 extension board as [%s]", data.cmd ? "enable" : "disable");
  E_OsdkStat ret = vehicle->linker->sendSync(&cmdInfo,
                                             (uint8_t *) &data,
                                             &ackInfo,
                                             cbData,
                                             1000,
                                             3);
  if (ret == OSDK_STAT_OK) {
    return true;
  } else if (ret == OSDK_STAT_ERR_TIMEOUT) {
    DERROR("Set timeout !");
    return false;
  } else {
    DERROR("Set error !");
    return false;
  }
}

bool ConfigTool::doProcessSimulationCtrl(std::string GPSLatInDeg, 
                                         std::string GPSLonInDeg, 
                                         std::string activeAction)
{
  bool action = false;
  if(activeAction.compare("on") == 0)
  {
    action = true;
  }
  else if(activeAction.compare("off") == 0)
  {
    action = false;
  }
  else
  {
    DERROR("Error: unknown simulation action!\n");
    return false;
  }

  std::string::size_type sz;
  double latitudeInDeg  = action ? std::stod(GPSLatInDeg, &sz) : 0;
  double longitudeInDeg = action ? std::stod(GPSLonInDeg, &sz) : 0;

  return vehicle->setSimulationOn(action, latitudeInDeg, longitudeInDeg);
}

int
main(int argc, char** argv)
{
  if(argc < 5)
  {
    show_usage(argv[0]);
    return 1;
  }

  // Setup OSDK.
  LinuxSetup *linuxEnvironment = new LinuxSetup(argc, argv);
  Vehicle *vehicle = linuxEnvironment->getVehicle();
  if (vehicle == NULL) {
    DERROR("Error: Vehicle not initialized!\n");
    return 1;
  }

  std::vector<std::string> sources;
  bool hasOption   = false;
  bool isLatitude  = false;
  bool isLongitude = false;
  std::string activeCMD;
  std::string gpsLatInDeg;
  std::string gpsLonInDeg;

  for (int i = 0; i < argc; i++) {
    std::string arg = argv[i];

    if (arg.compare("--help") == 0) {
      show_usage(argv[0]);
      return 0;
    }

    if ((arg.compare("--usb-port") == 0) ||
        (arg.compare("--config-file") == 0)) {
      if (i + 1 < argc) {
        hasOption = true;
        continue;
      } else {
        DERROR("%s option requires one argument.\n", arg.c_str());
      }
    } else if ((arg.compare("--power-supply") == 0) ||
        (arg.compare("--usb-connected-flight") == 0) ||
        (arg.compare("--simulation") == 0) ||
        (arg.compare("--latitude") == 0) ||
        (arg.compare("--longitude") == 0)) {
      if (i + 1 < argc) {
        hasOption = true;

        if (activeCMD.compare("--simulation") != 0) {
          activeCMD = arg;
        }

        if (arg.compare("--latitude") == 0) {
          isLatitude = true;
        } else if (arg.compare("--longitude") == 0) {
          isLongitude = true;
        }

        continue;
      } else {
        DERROR("%s option requires one argument.\n", arg.c_str());
      }
    } else {
      if (isLatitude) {
        gpsLatInDeg = argv[i];
        isLatitude = false;
      } else if (isLongitude) {
        gpsLonInDeg = argv[i];
        isLongitude = false;
      } else if (hasOption || i == 0) {
        sources.push_back(argv[i]);
      }
      hasOption = false;
    }
  }

  std::string configFile = "";
  std::string usbPort = linuxEnvironment->getEnvironment()->getDeviceAcm();
  std::string activeAction;
  std::string subStr("/dev/tty");

  for (uint32_t i = 0; i < sources.size(); i++) {
    std::ifstream ifs(argv[i]);
    if (ifs.is_open() && sources[i].find(subStr) == std::string::npos && (configFile == "")) {
      configFile = sources[i];
    } else if (sources[i].compare("on") == 0 ||
        sources[i].compare("off") == 0) {
      activeAction = sources[i];
    } else if (sources[i].find(subStr) != std::string::npos) {
      usbPort = sources[i];
    }
  }

  if (configFile.empty() ||
      usbPort.empty() ||
      activeAction.empty()) {
    DERROR("Error parsing your parameters! See --help option.\n");
    return 1;
  }

  // Initialize variables
  int functionTimeout = 1;
  char _func[100];

  // Obtain Control Authority
  ACK::ErrorCode ack = vehicle->control->obtainCtrlAuthority(functionTimeout);
  ACK::getErrorCodeMessage(ack, _func);

  ConfigTool* configTool = new ConfigTool(vehicle);
  if(configTool == 0)
  {
    DERROR("Error creating Config Tool! Check your USB port\n");
    return 1;
  }

  if(activeCMD.compare("--usb-connected-flight") == 0)
  {
    if(!configTool->doProcessUSBCtrl(activeAction))
    {
      DERROR("Error processing command\n");
      return 1;
    }
  }
  else if(activeCMD.compare("--power-supply") == 0)
  {
    if(!configTool->doProcessPowerSupplyCtrl(activeAction))
    {
      DERROR("Error processing command\n");
      return 1;
    }
  }
  else if(activeCMD.compare("--simulation") == 0)
  {
    if(activeAction.compare("on") == 0 && (gpsLatInDeg.empty() || gpsLonInDeg.empty()))
    {
      DERROR("Error: latitude or longitude options not set!\n");
      return 1;
    }

    if(!configTool->doProcessSimulationCtrl(gpsLatInDeg, gpsLonInDeg, activeAction))
    {
      DERROR("Error processing command\n");
      return 1;
    }
  }

  delete (configTool);
  return 0;
}
