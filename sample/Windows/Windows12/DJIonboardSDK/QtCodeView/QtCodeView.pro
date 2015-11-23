TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    ../DJIonboardSDK/WindowsHardDriver.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ../DJIonboardSDK/WindowsHardDriver.h

