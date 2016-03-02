/*! @brief *********************************
 * ****************************************** Copyright (c) ******************************************
 * @author: William Wu
 * @e-mail: DJI@justwillim.com
 * @date  : 2015-08-16
 * ****************************************** Copyright (c) ******************************************/
#include "cppforstm32.h"
#include "BspUsart.h"

#ifdef DYNAMIC_MEMORY
void * operator new (size_t size)
{
    if(NULL == size)
    {
#ifdef DEBUG
        printf("Error! Size is zero");
#endif//DEBUG
        return NULL;
    }
    void *p = malloc(size);
#ifdef DEBUG
    if(p == 0)
        printf("Lack Memory!");
#endif//DEBUG
    return p;
}

void * operator new [](size_t size)
{
    return operator new(size);
}

void operator delete (void * pointer)
{
    if(NULL != pointer)
    {
        free(pointer);
    }
}

void operator delete[](void * pointer)
{
    operator delete(pointer);
}
#endif //DYNAMIC_MEMORY


//!@code printf link functions
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
int fputc(int ch,FILE *f)
{	
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2,(unsigned char)ch);
	
	return (ch);
}
int GetKey(void)
{
   
}
#ifdef __cplusplus
}
#endif //__cplusplus
//!@endcode printf link fuctions.