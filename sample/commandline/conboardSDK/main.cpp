#include <QCoreApplication>
#include <iostream>

#include "conboardsdktask.h"
using namespace std;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    cout << "hello world";

    //! @note replace these two lines below to change to an other hard-driver level.
    QSerialPort *port = new QSerialPort();
    QHardDriver *driver = new QHardDriver(port);

    CoreAPI *api = new CoreAPI(driver);

    ConboardSDKScript scriptSDK(api);

    ScriptThread *st = new ScriptThread(&scriptSDK);

    //! @note replace these four lines below to change to an other hard-driver level.
    APIThread *send = new APIThread(api, 1, port);
    APIThread *read = new APIThread(api, 2, port);
    send->start();
    read->start();
    st->start();

    return a.exec();
}

