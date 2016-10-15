#include "LAS_LoggingTest.h"

void LAS_LoggingTest::SetUp() {
  LIDAR_APITest::SetUp();
}

void LAS_LoggingTest::TearDown() {
  LIDAR_APITest::TearDown();

  //TODO release mutex
}

TEST_F(LAS_LoggingTest, pcap2las_01) {
  path tmpFile;
  path searchPath(current_path().string() + "/../");

  if(findFile(searchPath, "tmpFile", tmpFile))
    ASSERT_EQ(remove(tmpFile.c_str()), 0) << "Error removing tmpFile\n";

  // Start all worker threads
  boost::lock_guard<boost::mutex> lock(io_mutex);
  boost::thread workerThread0(runSimulator);
  boost::thread workerThread1(udpAndLogging);
  boost::thread workerThread2(managementProcessing);

  // Wait until logging finished
  long fSize = getFileSize(tmpFile.string());
  do {
    fSize = getFileSize(tmpFile.string());
    sleep(3);
  }while(fSize != getFileSize(tmpFile.string()));

  // Stop all worker threads
  controlFlag = 1;
  condition.notify_one();
  workerThread0.interrupt();
  workerThread1.interrupt();
  workerThread2.interrupt();

  // Compare generated LAS file against pre-defined LAS file

  // Generated LAS file
  io::mapped_file_source f1(currLogFileLas.c_str());

  // LAS file to test against
  path baseLASFile;
  ASSERT_TRUE(findFile(searchPath, "VLP16_log01.las", baseLASFile));
  io::mapped_file_source f2(baseLASFile.c_str());

  bool res = f1.size() == f2.size() &&
      std::equal(f1.data(), f1.data() + f1.size(), f2.data()) ? 1 : 0;
  ASSERT_TRUE(res);
}

bool LAS_LoggingTest::findFile(const path& dir_path, const path& file_name, path& path_found) {
  const recursive_directory_iterator end;
  const auto it = find_if(recursive_directory_iterator(dir_path), end,
                          [&file_name](const directory_entry& e) {
                            return e.path().filename() == file_name;
                          });
  if (it == end) {
    return false;
  } else {
    path_found = it->path();
    return true;
  }
}

long LAS_LoggingTest::getFileSize(std::string filename) {
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

void LAS_LoggingTest::singalhandler(const boost::system::error_code& error, int signal_number) {
  if (!error)
    ctrlCOrCloseTerminalFlag = true;
}

void LAS_LoggingTest::runSimulator() {
  path simulatorCMD;
  path pcapFile;
  path searchPath(current_path().string() + "/../");
  std::string runCMD;
  // tmpFile is just a place holder of command output
  std::string options = " 0 127.0.0.1 0 >> tmpFile";

  // Find PCAP sample file
  ASSERT_TRUE(findFile(searchPath, "sample01.pcap", pcapFile));
  // Find Simulator executable
  ASSERT_TRUE(findFile(searchPath, "Simulator", simulatorCMD));

  runCMD = simulatorCMD.string() + "  " + pcapFile.string() + options;

  // Run Simulator
  std::system(runCMD.c_str());
}

void LAS_LoggingTest::udpAndLogging(void) {
  std::string logFilePathName="/home/";
  std::string logFilerFolderNamer="/Vlp16_logfiles/";
  std::string logFileDefaultNamer="Vlp16_log_";
  std::string logFilePcapExtension=".pcap";
  std::string logFileLasExtension=".las";
  std::string logFilePathNamePcap;
  std::string logFilePathNameLas;
  std::string currentUser;

  currentUser = getenv("USER");
  std::cout << "Current User =" << currentUser <<std::endl;

  logFilePathName.append(currentUser);

  logFilePathName.append(logFilerFolderNamer);

  if(!is_directory(logFilePathName.c_str()))
    ASSERT_TRUE(create_directory(logFilePathName.c_str()))
      << "Error creating directory\n";

  logFilePathName.append(logFileDefaultNamer);

  std::time_t rawTime;
  char tmpBuf[100];

  std::time(&rawTime);
  std::strftime(tmpBuf,100,"%H_%M_%S_%m_%d_%Y",std::localtime(&rawTime));
  std::cout << "Current Local time =";
  std::puts(tmpBuf);

  logFilePathName.append(tmpBuf);
  logFilePathNamePcap=logFilePathName;
  logFilePathNamePcap.append(logFilePcapExtension);
  logFilePathNameLas=logFilePathName;
  logFilePathNameLas.append(logFileLasExtension);

  std::cout << "Current Log file is in =";
  std::cout << logFilePathName <<std::endl;

  unsigned int printingToTerminal=1;   //0: turn it off

  currLogFileLas = logFilePathNameLas;
  try {
    boost::asio::io_service io_service1;
    UDPdriver udpDriver(io_service1,
			loggingTypeFlag,
			logFilePathNamePcap,
			logFilePathNameLas,
			printingToTerminal,
			&controlFlag);
    io_service1.run();

  }
  catch (std::exception& err) {
    std::cerr << err.what() << std::endl;
  }
}

void LAS_LoggingTest::managementProcessing(void) {
  boost::asio::io_service io_service2;
  boost::asio::signal_set signals(io_service2, SIGINT, SIGTERM, SIGHUP);

  signals.async_wait(LAS_LoggingTest::singalhandler);
  io_service2.run();
}
