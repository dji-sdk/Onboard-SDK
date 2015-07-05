#ifndef __DJI_PRO_HW_H__
#define __DJI_PRO_HW_H__

int Pro_Hw_Send(unsigned char *buf, int len);
int Pro_Hw_Recv(unsigned char *buf, int len);
int Pro_Hw_Setup(const char *device,int baudrate);

#endif

