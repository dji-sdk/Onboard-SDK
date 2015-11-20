#include <Windows.h>
#include <stdio.h>
#include "WindowsHardDriver.h"

void WinHardDriver::init()
{
	HANDLE hComm;
	hComm = CreateFile((LPCWSTR)"COM3",
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		0);
	if (hComm == INVALID_HANDLE_VALUE)
		printf("Fail to open port");
		// error opening port; abort
		;
}