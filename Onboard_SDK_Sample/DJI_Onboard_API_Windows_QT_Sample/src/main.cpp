#include "mainwindow.h"
#include <QApplication>
#include <unistd.h>
#include <qdebug.h>
#include <windows.h>
#include <stdio.h>
#include "DJI_Pro_Sample.h"

int main(int argc, char *argv[])
{
    AllocConsole();
    freopen("conin$","r+t",stdin);
    freopen("conout$","w+t",stdout);
    freopen("conout$","w+t",stderr);

    QApplication a(argc, argv);
    //MainWindow w;
   // w.show();
    MainWindow::Get_Instance()->show();

    DJI_Sample_Setup();

    return a.exec();
}
