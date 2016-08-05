QT += core
QT -= gui

CONFIG += c++11

TARGET = myboostQtTst
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    ../lidar_APIs_LIB/src/udpDrive.cpp \
    ../lidar_APIs_LIB/src/vtkPacketFileWriter.cxx

HEADERS += \
    ../lidar_APIs_LIB/inc/udpDriver.h \
    ../lidar_APIs_LIB/inc/vtkPacketFileWriter.h


INCLUDEPATH +=/path/boost/
INCLUDEPATH +=\
            ../lidar_APIs_LIB/inc


LIBS += -L/path/boost/lib/ -lboost_thread -lboost_system
LIBS +=-lpcap

