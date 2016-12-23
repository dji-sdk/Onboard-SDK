#include "DJI_Environment.h"

DJI_Environment::DJI_Environment(const std::string &config_file_path) {
  this->config_file_path = config_file_path;
  this->read_config_file();
}
DJI_Environment::~DJI_Environment() {}
void DJI_Environment::SetUp() {}
void DJI_Environment::TearDown() {}

/**
 * @note Find file within osdk-core directory
 */
std::string DJI_Environment::findFile(std::string file) {
  char cwd[1024];
  std::string jsonFile;

  if(getcwd(cwd, sizeof(cwd)) == NULL)
    throw std::runtime_error("Error getting current directory");

  std::string strCWD(cwd);
  jsonFile = strCWD + "/osdk-core/" + file;
  std::ifstream fStream(jsonFile.c_str());

  if(!fStream.good())
    jsonFile.clear();

  return jsonFile;
}

void DJI_Environment::read_config_file() {
  std::ifstream ifs(config_file_path.c_str());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));
  rapidjson::Document document;
  document.Parse(content.c_str());

  version = document["version"].GetString();
  app_id = document["api_id"].GetInt();
  enc_key = document["api_key"].GetString();
  device = document["device"].GetString();
  baudrate = document["baudrate"].GetUint();
  if (document.HasMember("sim_control_enabled")) {
    sim_control_enabled = document["sim_control_enabled"].GetBool();
    if (sim_control_enabled) {
      sim_ip = document["sim_ip"].GetString();
      sim_hash = document["sim_hash"].GetString();
      sim_path = document["sim_path"].GetString();
      sim_timeout_s = document["sim_timeout_s"].GetUint();
    }
  }
}

const std::string &DJI_Environment::getVersion() const { return version; }

int DJI_Environment::getApp_id() const { return app_id; }

const std::string &DJI_Environment::getEnc_key() const { return enc_key; }

const std::string &DJI_Environment::getSim_ip() const { return sim_ip; }

const std::string &DJI_Environment::getSim_hash() const { return sim_hash; }

const std::string &DJI_Environment::getSim_path() const { return sim_path; }

unsigned int DJI_Environment::getSim_timeout_s() const { return sim_timeout_s; }

bool DJI_Environment::isSim_control_enabled() const {
  return sim_control_enabled;
}

const std::string &DJI_Environment::getDevice() const { return device; }

unsigned int DJI_Environment::getBaudrate() const { return baudrate; }
