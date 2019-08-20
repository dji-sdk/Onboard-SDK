#ifndef ONBOARDSDK_CONFIG_H
#define ONBOARDSDK_CONFIG_H

#include <opencv2/core/core.hpp>
#include <iostream>

namespace M210_STEREO
{

class Config
{
private:
  Config();

public:
  ~Config();

  static Config& instance();

  static Config* instancePtr();

  static void setParamFile(const std::string& file_name);

  template <typename T>
  static T get(const std::string& key)
  {
    T t;
    Config::instancePtr()->file_[key] >> t;
    return t;
  }

private:
  static Config* single_instance_;

  cv::FileStorage file_;

};

} // namespace M210_STEREO


#endif //ONBOARDSDK_CONFIG_H
