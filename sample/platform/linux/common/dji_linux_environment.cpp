/*! @file dji_linux_environment.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Helper functions to handle user configuration parsing
 *
 *  @Copyright (c) 2017 DJI
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

#include "dji_linux_environment.hpp"

DJI_Environment::DJI_Environment(const std::string& config_file_path)
{

  this->config_file_path   = config_file_path;
  this->config_read_result = this->parse(config_file_path);
}
DJI_Environment::~DJI_Environment()
{
}

/**
 * @note Find file within osdk-core directory
 */
std::string
DJI_Environment::findFile(std::string file)
{
  char        cwd[1024];
  std::string configFile;

  if (getcwd(cwd, sizeof(cwd)) == NULL)
    throw std::runtime_error("Error getting current directory");

  std::string strCWD(cwd);
  // configFile = strCWD + "/osdk-core/" + file;
  configFile = strCWD + "/" + file; // just in the current working directory

  std::ifstream fStream(configFile.c_str());

  if (!fStream.good())
    configFile.clear();

  return configFile;
}

int
DJI_Environment::getApp_id() const
{
  return app_id;
}

const std::string&
DJI_Environment::getEnc_key() const
{
  return enc_key;
}

const std::string&
DJI_Environment::getDevice() const
{
  return device;
}

const std::string&
DJI_Environment::getDeviceAcm()  const
{
  return device_acm;
}

void
DJI_Environment::setDeviceAcm(std::string dev_path)
{
  device_acm = dev_path;
}

void
DJI_Environment::setSampleCase(std::string sample_case_name)
{
  sample_case = sample_case_name;
}

std::string
DJI_Environment::getSampleCase()
{
  return sample_case;
}

unsigned int
DJI_Environment::getBaudrate() const
{
  return baudrate;
}

unsigned int
DJI_Environment::getACMDefaultBaudrate() const
{
  return default_acm_baudrate;
}

bool
DJI_Environment::getConfigResult() const
{
  return config_read_result;
}

bool
DJI_Environment::parse(std::string config_file_path)
{
  char        line[1024];
  static char key[70];
  char        devName[20];
  char        acmName[20];

  bool setACM = false;
  bool setID = false, setKey = false, setBaud = false, setSerialDevice = false;
  bool result = false;

  std::ifstream read(config_file_path);

  if (read.is_open())
  {
    while (!read.eof())
    {
      read.getline(line, 1024);
      if (*line != 0) //! @note sscanf have features on empty buffer.
      {
        if (sscanf(line, "app_id : %d", &this->app_id))
        {
          std::cout << "Read App ID\n";
          setID = true;
        }
        if (sscanf(line, "app_key : %s", key))
        {
          this->enc_key = std::string(key);
          setKey        = true;
        }
        if (sscanf(line, "device : %s", devName))
        {
          this->device    = std::string(devName);
          setSerialDevice = true;
        }
        if (sscanf(line, "baudrate : %d", &this->baudrate))
        {
          setBaud = true;
        }
        if (sscanf(line, "acm_port : %s", acmName))
        {
          this->device_acm = std::string(acmName);
          setACM = true;
        }
      }
    }
    if (setBaud && setID && setKey && setSerialDevice)
    {
      std::cout << "User Configuration read successfully. \n\n";
      result = true;
    }
    else
    {
      std::cout << "There's an error with your UserConfig.txt file.\n";
      std::cout << "Recommended format of UserConfig.txt :\n"
                   "app_id : 123456\n"
                   "app_key : 0123456789abcdefghijklmnopqrstuvwxyz\n"
                   "device : /dev/ttyUSBx\n"
                   "baudrate : 921600\n"
                   "acm_port : /dev/ttyACMx\n\n";
      result = false;

    }

    if (!setACM)
    {
      std::cout << "Cannot get ACM device name !!! Some of these OSDK APIs will"
                   " be not supported :\n"
                   "   GimbalManager APIs\n"
                   "   CameraManager APIs\n"
                   "   Advance sensing APIs\n";
      std::cout << "Recommended format of UserConfig.txt :\n"
                   "app_id : 123456\n"
                   "app_key : 0123456789abcdefghijklmnopqrstuvwxyz\n"
                   "device : /dev/ttyUSBx\n"
                   "baudrate : 921600\n"
                   "acm_port : /dev/ttyACMx\n\n";
    }

    read.close();
  }
  else
  {
    std::cout << "User config file could not be opened. Make sure your "
                 "filepath is correct\n"
              << "and have sufficient permissions." << std::endl;
    result = false;
  }

  return result;
}
