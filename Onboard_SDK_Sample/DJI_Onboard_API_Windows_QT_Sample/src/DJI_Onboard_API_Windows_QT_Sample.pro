#-------------------------------------------------
#
# Project created by QtCreator 2015-04-22T15:49:49
#
#-------------------------------------------------

QT       += core gui
QT       += serialport

#win32:CONFIG(debug, debug|release):CONFIG += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DJI_Onboard_API_Windows_QT_Sample
TEMPLATE = app

QT += widgets
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x000000

RC_ICONS = SDK.ico

SOURCES += main.cpp\
        mainwindow.cpp \
    DJI_Pro_Hw.cpp \
    DJI_Pro_Codec.cpp \
    DJI_Pro_Link.cpp \
    DJI_Pro_App.cpp \
    DJI_Pro_Test.cpp \
    qextserialbase.cpp \
    qextserialport.cpp \
    win_qextserialport.cpp \
    about.cpp \
    DJI_Pro_Rmu.cpp

HEADERS  += mainwindow.h \
    DJI_Pro_Hw.h \
    DJI_Pro_Codec.h \
    DJI_Pro_Link.h \
    DJI_Pro_App.h \
    DJI_Pro_Test.h \
    qextserialbase.h \
    qextserialport.h \
    win_qextserialport.h \
    about.h \
    DJI_Pro_Rmu.h \
    DJI_Pro_Config.h

FORMS    += mainwindow.ui \
    about.ui

RESOURCES += \
    image.qrc
