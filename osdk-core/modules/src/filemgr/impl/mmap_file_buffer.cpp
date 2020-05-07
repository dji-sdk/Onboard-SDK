//
// Created by dji on 4/15/20.
//
#include "mmap_file_buffer.hpp"
#include "dji_log.hpp"
#include <string.h>

namespace DJI {
namespace OSDK {

MmapFileBuffer::MmapFileBuffer() : fdAddr(NULL) {}

MmapFileBuffer::~MmapFileBuffer() {}

bool MmapFileBuffer::init(std::string path, uint64_t fileSize) {
  currentLogFilePath = path;
  fdAddrSize = fileSize;
  printf("Preparing File : %s\n", this->currentLogFilePath.c_str());
  fd = open(this->currentLogFilePath.c_str(), O_RDWR | O_CREAT, 0644);
  DSTATUS("fd = %d", fd);
  if (fd < 0) return false;

  ftruncate(fd, fdAddrSize);
  fdAddr = (char *) mmap(NULL, fdAddrSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (fdAddr)
    return true;
  else
    return false;
}

bool MmapFileBuffer::deInit() {
  DSTATUS("Deinit");
  if (fd >= 0) close(fd);
  fd = -1;
  if (fdAddr) munmap(fdAddr, fdAddrSize);
  fdAddr = NULL;
  return true;
}

// flag 代表是否覆盖已有队列缓存
bool MmapFileBuffer::InsertBlock(const uint8_t *pack, uint32_t data_length, int index) {
  static uint32_t tempAdaptingBufferCnt = 0;
  //DSTATUS("data_length = %d index = %d", data_length, index);
  if (data_length != tempAdaptingBufferCnt) DSTATUS("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!data_length != 800  (%d)\n", data_length);
  tempAdaptingBufferCnt = data_length;

  if ((data_length <= 0) || !fdAddr) {
    return false;
  }
  index = index * tempAdaptingBufferCnt;
  memcpy(fdAddr + index, pack, data_length);

  return true;
}
}
}