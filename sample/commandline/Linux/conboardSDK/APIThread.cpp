#include "APIThread.h"

APIThread::APIThread()
{
    api = 0;
    type = 0;
}

APIThread::APIThread(CoreAPI *API, int Type)
{
    api = API;
    type = Type;
}

bool APIThread::createThread()
{
    pthread_t tid;
    int ret = -1;
    std::string infoStr;
    if(1 == type)
    {
        ret = pthread_create(&tid, NULL, send_call, (void*)api);
        infoStr = "sendPoll";
    }
    else if(2 == type)
    {
        ret = pthread_create(&tid, NULL, read_call, (void*)api);
        infoStr = "readPoll";
    }
    else
    {
        infoStr = "error type number";
    }
    if(0 != ret)
    {
        API_LOG(api->getDriver(), ERROR_LOG, "fail to create thread for %s!\n",
                infoStr.c_str());
        return false;
    }
    return true;
}

void *APIThread::send_call(void *param)
{
    while(true)
    {
        ((CoreAPI*)param)->sendPoll();
    }
}

void *APIThread::read_call(void *param)
{
    while(true)
    {
        ((CoreAPI*)param)->readPoll();
    }
}
