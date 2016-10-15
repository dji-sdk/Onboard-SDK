#include "DJI_Common.h"
#include "DJI_Environment.h"
#include "gtest/gtest.h"

DJI_Environment* environment;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  std::string config_file_path = "../config.json";
  if (argc > 1) {
    config_file_path = argv[1];
  }

  environment = new DJI_Environment(config_file_path);
  testing::AddGlobalTestEnvironment(environment);

  return RUN_ALL_TESTS();
}
