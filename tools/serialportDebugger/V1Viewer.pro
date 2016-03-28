#-------------------------------------------------
#
# Project created by QtCreator 2016-03-01T22:20:38
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = V1Viewer
TEMPLATE = app


SOURCES += main.cpp\
        v1veiwer.cpp \
    QHardDriver.cpp

HEADERS  += v1veiwer.h \
    QHardDriver.h

FORMS    += v1veiwer.ui
