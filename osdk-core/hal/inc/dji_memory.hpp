/** @file dji_memory.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Implement memory management for DJI OSDK .
 *
 *  @Copyright (c) 2016-2017 DJI
 *
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

#ifndef DJI_MEMORY_H
#define DJI_MEMORY_H

#include "dji_type.hpp"

namespace DJI
{
namespace OSDK
{

#define PRO_PURE_DATA_MAX_SIZE 1007 // 2^10 - header size

class MMU
{
public:
  MMU();
  void setupMMU(void);
  void freeMemory(MMU_Tab* mmu_tab);
  MMU_Tab* allocMemory(uint16_t size);

public:
  static const int MMU_TABLE_NUM = 32;
  static const int MEMORY_SIZE   = 1024;

private:
  MMU_Tab memoryTable[MMU_TABLE_NUM];
  uint8_t memory[MEMORY_SIZE];
};

} // OSDK
} // DJI
#endif // DJI_MEMORY_H
