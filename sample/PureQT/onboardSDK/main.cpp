#include "djionboardsdk.h"
#include <QApplication>
#include <stdio.h>
#include <QDebug>
#include "QonboardSDK.h"

int main(int argc, char *argv[])
{
    qDebug()<<sizeof(Header);

    QApplication a(argc, argv);
    DJIonboardSDK w;
    w.show();

    return a.exec();
}
