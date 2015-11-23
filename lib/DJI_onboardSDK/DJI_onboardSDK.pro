TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ../inc/DJI_API.h \
    ../inc/DJI_App.h \
    ../inc/DJI_Codec.h \
    ../inc/DJI_Config.h \
    ../inc/DJI_HardDriver.h \
    ../inc/DJI_Link.h \
    ../inc/DJI_Memory.h \
    ../inc/DJI_Type.h \
    ../inc/DJI_FlightContorl.h

INCLUDEPATH +=\
    ../inc

