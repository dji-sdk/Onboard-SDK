#include "DJI_Common.h"
#include "DJI_Environment.h"

DJI_Environment* environment;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  std::string config_file_path;

  if (argc > 1) {
    config_file_path = argv[1];
    std::ifstream fStream(config_file_path.c_str());
    if(!fStream.good())
      throw std::runtime_error("User configuration file not found");
  }else {
    config_file_path = environment->findFile("config.json");
    if(config_file_path.empty())
      throw std::runtime_error("User configuration file not found");
  }

  environment = new DJI_Environment(config_file_path);

  testing::AddGlobalTestEnvironment(environment);

  return RUN_ALL_TESTS();
}
