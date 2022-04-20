//
// Created by dji on 4/15/20.
//

#ifndef ONBOARDSDK_INTERNAL_RESOURCES_SRC_OSDK_CORE_MODULES_INC_FILEMGR_IMPL_MMAP_FILE_BUFFER_HPP_
#define ONBOARDSDK_INTERNAL_RESOURCES_SRC_OSDK_CORE_MODULES_INC_FILEMGR_IMPL_MMAP_FILE_BUFFER_HPP_

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include <atomic>
#include <string>

namespace DJI {
namespace OSDK {
class MmapFileBuffer {
 public:
  MmapFileBuffer();
  ~MmapFileBuffer();

  std::string currentLogFilePath;
  int fd;
  char *fdAddr;
  uint64_t fdAddrSize;
  uint64_t curFilePos;

  bool init(std::string path, uint64_t fileSize);

  bool deInit();

  bool InsertBlock(const uint8_t *pack, uint32_t data_length, uint64_t index);

};
}
}
#endif //ONBOARDSDK_INTERNAL_RESOURCES_SRC_OSDK_CORE_MODULES_INC_FILEMGR_IMPL_MMAP_FILE_BUFFER_HPP_
