TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ../include/DJI_API.h \
    ../include/DJI_App.h \
    ../include/DJI_Codec.h \
    ../include/DJI_Config.h \
    ../include/DJI_HardDriver.h \
    ../include/DJI_Link.h \
    ../include/DJI_Memory.h \
    ../include/DJI_Type.h

INCLUDEPATH +=\
    ../include
