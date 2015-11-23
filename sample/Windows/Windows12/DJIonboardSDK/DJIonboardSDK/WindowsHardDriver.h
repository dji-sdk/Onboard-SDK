#ifndef WINDOWSHARDDRIBER
#define WINDOWSHARDDRIBER
/*
#include "DJI_HardDriver.h"

class WinHardDriver : public DJI::onboardSDK::HardDriver
{
  public:
	WinHardDriver();

  public:
	void init();
	unsigned int getTimeStamp();
	size_t send(const uint8_t *buf, size_t len);
	size_t readall(uint8_t *buf, size_t maxlen);

  public:
	void lockMemory();
	void freeMemory();

	void lockMSG();
	void freeMSG();
};
*/
#endif // WINDOWSHARDDRIBER

#ifndef _COM_H_
#define _COM_H_

#include <cassert>
#include <strstream>
#include <algorithm>
#include <exception>
#include <iomanip>
using namespace std;
#include <windows.h>

class _baseCom
{
  protected:
    volatile HANDLE _com_handle;
    volatile int _port;
    char _com_str[50];
    COMMTIMEOUTS _co;
    DCB _dcb;

    void init();
    virtual bool openPort() = 0;
    virtual bool setup_port();
    inline void setComPort(int port);

  public:
    _baseCom() { init(); }
    virtual ~_baseCom() { close(); }
    bool setState(char *set_str);

    bool setState(int BaudRate, int ByteSize = 8, int Parity = NOPARITY,
                  int StopBits = ONESTOPBIT);
    inline bool open(int port);
    bool open(int port, int baud_rate);
    inline bool open(int port, char *set_str);
    bool setBuf(int in, int out);
    inline virtual void close();
    inline bool isOpen();
    HANDLE getHandle() { return _com_handle; }
    operator HANDLE() { return _com_handle; }
};

class _syncCom : public _baseCom
{
  protected:
    virtual bool openPort();

  public:
    _syncCom() {}
    int read(char *buf, int buf_len);
    int write(char *buf, int buf_len);
    inline int write(char *buf);
    template <typename T> _syncCom &operator<<(T x)
    {
        strstream s;

        s << x;
        write(s.str(), s.pcount());

        return *this;
    }
};

class _asynCom : public _baseCom
{
  protected:
    OVERLAPPED _ro, _wo;
    virtual bool openPort();

  public:
    _asynCom();
    virtual ~_asynCom();
    int read(char *buf, int buf_len, int time_wait = 20);
    int write(char *buf, int buf_len);
    inline int write(char *buf);
    template <typename T> _asynCom &operator<<(T x)
    {
        strstream s;

        s << x;
        write(s.str(), s.pcount());

        return *this;
    }
};

#define ON_COM_RECEIVE WM_USER + 618

class _threadCom : public _asynCom
{
  protected:
    volatile HANDLE _threadHandle;
    volatile HWND _notifyHwnd;
    volatile long _notifyNum;
    volatile bool _runFlag;
    void (*_func)(int port);
    OVERLAPPED _waitO;

    virtual void onReceive();
    virtual bool openPort();

  public:
    _threadCom();
    ~_threadCom();
    void setNotifyNum(int num);
    int getNotifyNum() { return _notifyNum; }
    inline void setHwnd(HWND hWnd);
    inline HWND getHwnd() { return _notifyHwnd; }
    inline void setFunc(void (*f)(int)) { _func = f; }
    virtual void close();

    HANDLE getThread() { return _threadHandle; }
    bool suspend();
    bool resume();
    bool restart();

  private:
    static DWORD WINAPI comThread(LPVOID para);
};

typedef _threadCom _com;

#endif //_COM_H
