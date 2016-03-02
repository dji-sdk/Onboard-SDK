/*! @brief *********************************
 * ****************************************** Copyright (c) ******************************************
 * @author: William Wu
 * @e-mail: DJI@justwillim.com
 * @date  : 2015-08-16
 * ****************************************** Copyright (c) ******************************************/
#ifndef CPPFORSTM32_H
#define CPPFORSTM32_H
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#ifdef DYNAMIC_MEMORY
void * operator new (size_t size);
void * operator new [](size_t size);
void operator delete (void * pointer);
void operator delete[](void * pointer);
#endif //DYNAMIC_MEMORY
#endif // CPPFORSTM32_H
