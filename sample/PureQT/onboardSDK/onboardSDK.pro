#-------------------------------------------------
#
# Project created by QtCreator 2015-11-23T16:26:43
#
#-------------------------------------------------

QT       += core gui serialport  webkitwidgets

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
    ../../../lib/src/DJI_VirtualRC.cpp \
    ../../../lib/src/DJI_Camera.cpp \
    ../../../lib/src/DJI_HotPoint.cpp \
    ../../../lib/src/DJI_Mission.cpp \
    ../../../lib/src/DJI_Follow.cpp \
    ../../../lib/src/DJI_WayPoint.cpp \
    ../../../lib/DJIscript/src/DJI_Script.cpp \
    ../../../lib/DJIscript/src/DJI_Interpreter.cpp \
    highlighter.cpp \
    ../../../lib/gridmap/src/gridmapKernel.cpp \
    ../../../lib/DJIscript/src/cmdCamera.cpp \
    ../../../lib/DJIscript/src/cmdCoreAPI.cpp \
    ../../../lib/DJIscript/src/cmdFlight.cpp \
    ../../../lib/DJIscript/src/cmdFollow.cpp \
    ../../../lib/DJIscript/src/cmdHotPoint.cpp \
    ../../../lib/DJIscript/src/cmdIO.cpp \
    ../../../lib/DJIscript/src/cmdSettings.cpp \
    ../../../lib/DJIscript/src/cmdVirtualRC.cpp \
    ../../../lib/DJIscript/src/cmdWayPoint.cpp \
    conboardsdktask.cpp \
    ../../../lib/gridmap/src/gridmap.cpp

HEADERS  += djionboardsdk.h \
    QonboardSDK.h \
    ../../../lib/inc/DJI_API.h \
    ../../../lib/inc/DJI_App.h \
    ../../../lib/inc/DJI_Codec.h \
    ../../../lib/inc/DJI_Config.h \
    ../../../lib/inc/DJI_HardDriver.h \
    ../../../lib/inc/DJI_Link.h \
    ../../../lib/inc/DJI_Memory.h \
    ../../../lib/inc/DJI_Type.h \
    ../../../lib/inc/DJI_Version.h \
    ../../../lib/inc/DJI_Camera.h \
    ../../../lib/inc/DJI_Flight.h \
    ../../../lib/inc/DJI_VirtualRC.h \
    ../../../lib/inc/DJI_HotPoint.h \
    ../../../lib/inc/DJI_Mission.h \
    ../../../lib/inc/DJI_Follow.h \
    ../../../lib/inc/DJI_WayPoint.h \
    ../../../lib/DJIscript/inc/DJI_Script.h \
    ../../../lib/DJIscript/inc/DJI_Interpreter.h \
    ../../../lib/gridmap/inc/gridmapKernel.h \
    ../../../lib/DJIscript/inc/cmdCamera.h \
    ../../../lib/DJIscript/inc/cmdCoreAPI.h \
    ../../../lib/DJIscript/inc/cmdFlight.h \
    ../../../lib/DJIscript/inc/cmdFollow.h \
    ../../../lib/DJIscript/inc/cmdHotPoint.h \
    ../../../lib/DJIscript/inc/cmdIO.h \
    ../../../lib/DJIscript/inc/cmdSettings.h \
    ../../../lib/DJIscript/inc/cmdVirtualRC.h \
    ../../../lib/DJIscript/inc/cmdWayPoint.h \
    conboardsdktask.h \
    ../../../lib/gridmap/inc/gridmap.h

FORMS    += djionboardsdk.ui

INCLUDEPATH += \
            ../../../lib/inc \
            ../../../lib/DJIscript/inc \
            ../../../lib/gridmap/inc \

DEFINES += QT

RESOURCES += \
    ../../../lib/gridmap/src/shaders.qrc \
    ../../../lib/gridmap/src/textures.qrc

DISTFILES += \
    ../../../lib/gridmap/src/fshader.glsl \
    ../../../lib/gridmap/src/vshader.glsl
