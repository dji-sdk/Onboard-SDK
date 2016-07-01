#include "djionboardsdk.h"
#include <QApplication>
#include <stdio.h>
#include <QDebug>
#include "QonboardSDK.h"

int main(int argc, char *argv[])
{
    /******/
    qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");

   /******/

    QApplication a(argc, argv);
    DJIonboardSDK w;
    w.showMaximized();

    return a.exec();
}
