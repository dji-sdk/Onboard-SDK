/*! @file cppforstm32.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief Support for printf to USART2 on STM32 platform
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
 
#include "stm32f4xx.h"
#include "cppforstm32.h"
#include "BspUsart.h"

#ifdef DYNAMIC_MEMORY
void*
operator new(size_t size)
{
  if (NULL == size)
  {
#ifdef DEBUG
    printf("Error! Size is zero");
#endif // DEBUG
    return NULL;
  }
  void* p = malloc(size);
#ifdef DEBUG
  if (p == 0)
    printf("Lack Memory!");
#endif // DEBUG
  return p;
}

void*
operator new[](size_t size)
{
  return operator new(size);
}

void
operator delete(void* pointer)
{
  if (NULL != pointer)
  {
    free(pointer);
  }
}

void
operator delete[](void* pointer)
{
  operator delete(pointer);
}
#endif // DYNAMIC_MEMORY

//!@code printf link functions
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
// int fputc(int ch, FILE *f)
PUTCHAR_PROTOTYPE
{
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    ;
  USART_SendData(USART2, (uint8_t)ch);

  return (ch);
}
#ifdef __cplusplus
}
#endif //__cplusplus
//!@endcode printf link fuctions.
