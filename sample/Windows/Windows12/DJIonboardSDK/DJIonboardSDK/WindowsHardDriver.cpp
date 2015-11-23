//
#include <Windows.h>
#include <stdio.h>
#include "WindowsHardDriver.h"
/*
void WinHardDriver::init()
{
HANDLE hComm;
hComm = CreateFile((LPCWSTR) "COM3", GENERIC_READ | GENERIC_WRITE, 0, 0,
OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
if (hComm == INVALID_HANDLE_VALUE)
printf("Fail to open port");
// error opening port; abort
;
if (SetupComm(hComm, 2048, 2048))
printf("SetError");
}*/

void _baseCom::init() //鍒濆鍖?
{
	memset(_com_str, 0, 20);
	memset(&_co, 0, sizeof(_co));
	memset(&_dcb, 0, sizeof(_dcb));
	_dcb.DCBlength = sizeof(_dcb);
	_com_handle = INVALID_HANDLE_VALUE;
}

bool _baseCom::setup_port()
{
	if (!isOpen())
		return false;

	if (!SetupComm(_com_handle, 8192, 8192))
		return false; //璁剧疆鎺ㄨ崘缂撳啿鍖?

	if (!GetCommTimeouts(_com_handle, &_co))
		return false;
	_co.ReadIntervalTimeout = 0xFFFFFFFF;
	_co.ReadTotalTimeoutMultiplier = 0;
	_co.ReadTotalTimeoutConstant = 0;
	_co.WriteTotalTimeoutMultiplier = 0;
	_co.WriteTotalTimeoutConstant = 2000;
	if (!SetCommTimeouts(_com_handle, &_co))
		return false; //璁剧疆瓒呮椂鏃堕棿

	if (!PurgeComm(_com_handle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR |
                                    PURGE_RXCLEAR))
		return false; //娓呯┖涓插彛缂撳啿鍖?

	return true;
}

bool _baseCom::setState(char *set_str)
{
	if (isOpen())
	{
		if (!GetCommState(_com_handle, &_dcb))
			return false;
		if (!BuildCommDCB((LPCWSTR)set_str, &_dcb))
			return false;
		return SetCommState(_com_handle, &_dcb) == TRUE;
	}
	return false;
}

bool _baseCom::setState(int BaudRate, int ByteSize, int Parity, int StopBits)
{
	if (isOpen())
	{
		if (!GetCommState(_com_handle, &_dcb))
			return false;
		_dcb.BaudRate = BaudRate;
		_dcb.ByteSize = ByteSize;
		_dcb.Parity = Parity;
		_dcb.StopBits = StopBits;
		return SetCommState(_com_handle, &_dcb) == TRUE;
	}
	return false;
}

bool _baseCom::open(int port) { return open(port, 9600); }

bool _baseCom::open(int port, int baud_rate)
{
	if (port < 1 || port > 1024)
		return false;

	setComPort(port);

	if (!openPort())
		return false;

	if (!setup_port())
		return false;

	return setState(baud_rate);
}

bool _baseCom::open(int port, char *set_str)
{
	if (port < 1 || port > 1024)
		return false;

	setComPort(port);

	if (!openPort())
		return false;

	if (!setup_port())
		return false;

	return setState(set_str);
}

bool _baseCom::setBuf(int in, int out)
{
	return isOpen() ? SetupComm(_com_handle, in, out) : false;
}

void _baseCom::close()
{
	if (isOpen())
	{
		CloseHandle(_com_handle);
		_com_handle = INVALID_HANDLE_VALUE;
	}
}

bool _baseCom::isOpen() { return _com_handle != INVALID_HANDLE_VALUE; }

void _baseCom::setComPort(int port)
{
	char p[12];
	_port = port;
	strcpy(_com_str, "\\\\.\\COM");
	ltoa(_port, p, 10);
	strcat(_com_str, p);
}

bool _syncCom::openPort()
{
	if (isOpen())
		close();

	_com_handle = CreateFile((LPCWSTR)_com_str, GENERIC_READ | GENERIC_WRITE, 0,
                             NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	assert(isOpen());
	return isOpen(); //妫€娴嬩覆鍙ｆ槸鍚︽垚鍔熸墦寮€
}

int _syncCom::read(char *buf, int buf_len)
{
	if (!isOpen())
		return 0;

	buf[0] = '\0';

	COMSTAT stat;
	DWORD error;

	if (ClearCommError(_com_handle, &error, &stat) && error > 0) //娓呴櫎閿欒
	{
		PurgeComm(_com_handle, PURGE_RXABORT | PURGE_RXCLEAR);
		return 0;
	}

	unsigned long r_len = 0;

	buf_len = min(buf_len - 1, (int)stat.cbInQue);
	if (!ReadFile(_com_handle, buf, buf_len, &r_len, NULL))
		r_len = 0;
	buf[r_len] = '\0';

	return r_len;
}

int _syncCom::write(char *buf, int buf_len)
{
	if (!isOpen() || !buf)
		return 0;

	DWORD error;
	if (ClearCommError(_com_handle, &error, NULL) && error > 0) //娓呴櫎閿欒
		PurgeComm(_com_handle, PURGE_TXABORT | PURGE_TXCLEAR);

	unsigned long w_len = 0;
	if (!WriteFile(_com_handle, buf, buf_len, &w_len, NULL))
		w_len = 0;

	return w_len;
}

int _syncCom::write(char *buf)
{
	assert(buf);
	return write(buf, strlen(buf));
}

bool _asynCom::openPort()
{
	if (isOpen())
		close();

	_com_handle = CreateFile(
		(LPCWSTR)_com_str, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		NULL);
	assert(isOpen());
    return isOpen();
}

_asynCom::_asynCom()
{
	memset(&_ro, 0, sizeof(_ro));
	memset(&_wo, 0, sizeof(_wo));

	_ro.hEvent = CreateEvent(NULL, true, false, NULL);
	assert(_ro.hEvent != INVALID_HANDLE_VALUE);

	_wo.hEvent = CreateEvent(NULL, true, false, NULL);
	assert(_wo.hEvent != INVALID_HANDLE_VALUE);
}

_asynCom::~_asynCom()
{
	close();

	if (_ro.hEvent != INVALID_HANDLE_VALUE)
		CloseHandle(_ro.hEvent);

	if (_wo.hEvent != INVALID_HANDLE_VALUE)
		CloseHandle(_wo.hEvent);
}

int _asynCom::read(char *buf, int buf_len, int time_wait)
{
	if (!isOpen())
		return 0;

	buf[0] = '\0';

	COMSTAT stat;
	DWORD error;

	if (ClearCommError(_com_handle, &error, &stat) && error > 0)
	{
		PurgeComm(_com_handle, PURGE_RXABORT | PURGE_RXCLEAR);
		return 0;
	}

	if (!stat.cbInQue)
		return 0;

	unsigned long r_len = 0;

	buf_len = min((int)(buf_len - 1), (int)stat.cbInQue);

	if (!ReadFile(_com_handle, buf, buf_len, &r_len, &_ro))
	{
		if (GetLastError() == ERROR_IO_PENDING) // 缁撴潫寮傛I/O
		{
			// WaitForSingleObject(_ro.hEvent, time_wait); //绛夊緟20ms
			if (!GetOverlappedResult(_com_handle, &_ro, &r_len, false))
			{
				if (GetLastError() != ERROR_IO_INCOMPLETE) //鍏朵粬閿欒
					r_len = 0;
			}
		}
		else
			r_len = 0;
	}

	buf[r_len] = '\0';
	return r_len;
}

int _asynCom::write(char *buf, int buf_len)
{
	if (!isOpen())
		return 0;

	DWORD error;
	if (ClearCommError(_com_handle, &error, NULL) && error > 0) //娓呴櫎閿欒
		PurgeComm(_com_handle, PURGE_TXABORT | PURGE_TXCLEAR);

	unsigned long w_len = 0, o_len = 0;
	if (!WriteFile(_com_handle, buf, buf_len, &w_len, &_wo))
        if (GetLastError() != ERROR_IO_PENDING)
            w_len = 0;

	return w_len;
}

int _asynCom::write(char *buf)
{
	assert(buf);
	return write(buf, strlen(buf));
}

void _threadCom::onReceive()
{
	if (_notifyHwnd)
        ;//PostMessage(_notifyHwnd, ON_COM_RECEIVE, WPARAM(_port), LPARAM(0));
	else
	{
		if (_func)
			_func(_port);
	}
}

bool _threadCom::openPort()
{
	if (_asynCom::openPort())
	{
		_runFlag = true;
		DWORD id;
		_threadHandle =
            CreateThread(NULL, 0, comThread, this, 0, &id);
		assert(_threadHandle);
		if (!_threadHandle)
		{
			CloseHandle(_com_handle);
			_com_handle = INVALID_HANDLE_VALUE;
		}
		else
			return true;
	}
	return false;
}

_threadCom::_threadCom()
{
	_notifyNum = 0;
	_notifyHwnd = NULL;
	_threadHandle = NULL;
	_func = NULL;

	memset(&_waitO, 0, sizeof(_waitO));
	_waitO.hEvent = CreateEvent(NULL, true, false, NULL);
	assert(_waitO.hEvent != INVALID_HANDLE_VALUE);
}

_threadCom::~_threadCom()
{
	close();

	if (_waitO.hEvent != INVALID_HANDLE_VALUE)
		CloseHandle(_waitO.hEvent);
}

void _threadCom::setNotifyNum(int num) { _notifyNum = num; }

void _threadCom::setHwnd(HWND hWnd)
{
    _notifyHwnd = hWnd;
}

void _threadCom::close()
{
    if (isOpen())
    {
        _runFlag = false;
        SetCommMask(_com_handle, 0);
		SetEvent(_waitO.hEvent);

		if (WaitForSingleObject(_threadHandle, 100) != WAIT_OBJECT_0)
			TerminateThread(_threadHandle, 0);

		CloseHandle(_com_handle);
		CloseHandle(_threadHandle);

		_threadHandle = NULL;
		_com_handle = INVALID_HANDLE_VALUE;
		ResetEvent(_waitO.hEvent);
	}
}

bool _threadCom::suspend()
{
	return _threadHandle != NULL ? SuspendThread(_threadHandle) != 0xFFFFFFFF
                                 : false;
}

bool _threadCom::resume()
{
	return _threadHandle != NULL ? ResumeThread(_threadHandle) != 0xFFFFFFFF
                                 : false;
}

bool _threadCom::restart()
{
	if (_threadHandle)
    {
        _runFlag = false;
        SetCommMask(_com_handle, 0);
        SetEvent(_waitO.hEvent);

        if (WaitForSingleObject(_threadHandle, 100) != WAIT_OBJECT_0)
			TerminateThread(_threadHandle, 0);

        CloseHandle(_threadHandle);

        _runFlag = true;
        _threadHandle = NULL;

        DWORD id;
        _threadHandle = CreateThread(NULL, 0, comThread, this, 0, &id);
        return (_threadHandle != NULL); //杈呭姪绾跨▼//
    }
    return false;
}

DWORD _threadCom::comThread(LPVOID para)
{
    _threadCom *pcom = (_threadCom *)para;

    if (!SetCommMask(pcom->_com_handle, EV_RXCHAR | EV_ERR))
        return 0;

    COMSTAT stat;
    DWORD error;

    for (DWORD length, mask = 0; pcom->_runFlag && pcom->isOpen(); mask = 0)
    {
        if (!WaitCommEvent(pcom->_com_handle, &mask, &pcom->_waitO))
        {
			if (GetLastError() == ERROR_IO_PENDING)
			{
                GetOverlappedResult(pcom->_com_handle, &pcom->_waitO, &length,
                                    true);
			}
        }

        if (mask & EV_ERR) // == EV_ERR
			ClearCommError(pcom->_com_handle, &error, &stat);

        if (mask & EV_RXCHAR) // == EV_RXCHAR
        {
			ClearCommError(pcom->_com_handle, &error, &stat);
			if (stat.cbInQue > pcom->_notifyNum)
                pcom->onReceive();
        }
    }
    return 0;
}
