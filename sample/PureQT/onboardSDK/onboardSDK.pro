#-------------------------------------------------
#
# Project created by QtCreator 2015-11-23T16:26:43
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = onboardSDK
TEMPLATE = app


SOURCES += main.cpp\
        djionboardsdk.cpp \
    QonboardSDK.cpp \
    ../../../lib/src/DJI_API.cpp \
    ../../../lib/src/DJI_App.cpp \
    ../../../lib/src/DJI_Codec.cpp \
    ../../../lib/src/DJI_Flight.cpp \
    ../../../lib/src/DJI_HardDriver.cpp \
    ../../../lib/src/DJI_Link.cpp \
    ../../../lib/src/DJI_Memory.cpp \
    ../../../lib/src/DJI_VirtualRC.cpp

HEADERS  += djionboardsdk.h \
    QonboardSDK.h \
    ../../../lib/inc/DJI_API.h \
    ../../../lib/inc/DJI_App.h \
    ../../../lib/inc/DJI_Codec.h \
    ../../../lib/inc/DJI_Config.h \
    ../../../lib/inc/DJI_HardDriver.h \
    ../../../lib/inc/DJI_Link.h \
    ../../../lib/inc/DJI_Memory.h \
    ../../../lib/inc/DJI_Type.h

FORMS    += djionboardsdk.ui

INCLUDEPATH += \
            ../../../lib/inc \

DEFINES += QT
