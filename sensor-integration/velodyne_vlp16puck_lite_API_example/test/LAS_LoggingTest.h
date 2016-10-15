#include "LIDAR_APITest.h"
#include "udpDriver.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdio.h>

#include <sys/stat.h>
#include <algorithm>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/filesystem.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#define BOOST_FILESYSTEM_VERSION 3

using namespace std;
using namespace boost::filesystem;
namespace io = boost::iostreams;

boost::mutex io_mutex;
unsigned int controlFlag=0;
boost::condition_variable condition;

/**
 * 0: pcap only
 * 1: las file only
 * 2: both pcap and las files
 */
unsigned int loggingTypeFlag=2;
bool ctrlCOrCloseTerminalFlag = false;
std::string currLogFileLas;

class LAS_LoggingTest : public LIDAR_APITest {
 protected:
  virtual void SetUp();
  virtual void TearDown();

  // File management
  static bool findFile(const path& dir_path, const path& file_name, path& path_found);
  static long getFileSize(std::string filename);

  static void runSimulator();
  static void singalhandler(const boost::system::error_code& error, int signal_number);
  static void udpAndLogging(void);
  static void managementProcessing(void);
};
