#include "djionboardsdk.h"
#include <QApplication>
#include <stdio.h>
#include <QDebug>
#include "QonboardSDK.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DJIonboardSDK w;
    w.showMaximized();

    return a.exec();
}
