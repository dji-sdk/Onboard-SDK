QT += core
QT -= gui

CONFIG += c++11

TARGET = velodyne_lidar_sim
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    src/main.cpp \
    src/PacketFileSender.cxx \
    src/vvPacketSender.cxx

HEADERS += \
    inc/packetfilesender.h \
    inc/vtkPacketFileReader.h \
    inc/vvPacketSender.h

INCLUDEPATH +=\
            ./inc

LIBS += -L/path/boost/lib/ -lboost_thread -lboost_system
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
INCLUDEPATH +=/path/boost/

LIBS +=-lpcap

INCLUDEPATH += /usr/local/include/vtk-7.0
INCLUDEPATH += /opt/vtk-7.0.0
