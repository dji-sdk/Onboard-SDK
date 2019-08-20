/*
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

#include "config.hpp"

using namespace M210_STEREO;

Config* Config::single_instance_ = NULL;

Config::Config()
{
}

Config::~Config()
{
  if (file_.isOpened())
  {
    file_.release();
  }
}

Config&
Config::instance()
{
  return *Config::single_instance_;
}

Config*
Config::instancePtr()
{
  return Config::single_instance_;
}

void
Config::setParamFile(const std::string& file_name)
{
  if(!Config::single_instance_)
  {
    Config::single_instance_ = new Config();
  }

  Config::instancePtr()->file_ = cv::FileStorage( file_name, cv::FileStorage::READ );

  if(!Config::instancePtr()->file_.isOpened())
  {
    std::cerr << "Failed to open " << file_name << " file\n";
    Config::instancePtr()->file_.release();
  }
}
