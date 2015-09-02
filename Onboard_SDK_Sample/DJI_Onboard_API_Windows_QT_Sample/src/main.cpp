#include "mainwindow.h"
#include <QApplication>
#include <unistd.h>
#include <qdebug.h>
#include <windows.h>
#include <stdio.h>
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_Test.h"


int activation_callback_flag=0;
int main(int argc, char *argv[])
{
    AllocConsole();
    freopen("conin$","r+t",stdin);
    freopen("conout$","w+t",stdout);
    freopen("conout$","w+t",stderr);


    QApplication a(argc, argv);
    MainWindow w;
    w.show();


    DJI_Pro_Test_Setup();
    return a.exec();

}
