#ifndef ONBOARDSDK_DJI_ENVIRONMENT_H
#define ONBOARDSDK_DJI_ENVIRONMENT_H

#include <fstream>
#include <ostream>
#include "gtest/gtest.h"
#include "rapidjson/document.h"

class DJI_Environment : public testing::Environment {
 public:
  DJI_Environment(const std::string &config_file_path);
  virtual ~DJI_Environment();
  virtual void SetUp();
  virtual void TearDown();
  std::string findFile(std::string file);
  const std::string &getVersion() const;
  int getApp_id() const;
  const std::string &getEnc_key() const;
  const std::string &getSim_ip() const;
  const std::string &getSim_hash() const;
  const std::string &getSim_path() const;
  unsigned int getSim_timeout_s() const;
  bool isSim_control_enabled() const;
  const std::string &getDevice() const;
  unsigned int getBaudrate() const;

 private:
  void read_config_file();
  std::string config_file_path;
  std::string version;
  int app_id;
  std::string enc_key;
  std::string sim_ip;
  std::string sim_hash;
  std::string sim_path;
  unsigned int sim_timeout_s;
  bool sim_control_enabled;
  std::string device;
  unsigned int baudrate;
};

#endif  // ONBOARDSDK_DJI_ENVIRONMENT_H
