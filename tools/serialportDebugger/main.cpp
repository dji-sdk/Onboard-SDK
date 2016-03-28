#include "v1veiwer.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    V1Veiwer w;
    w.show();

    return a.exec();
}
