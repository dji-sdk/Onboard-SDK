TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    ../MDK/RTE/Device/STM32F407ZETx/system_stm32f4xx.c \
    ../MDK/RTE/Device/STM32F407ZETx/STCubeGenerated/Src/main.c \
    ../MDK/RTE/Device/STM32F407ZETx/STCubeGenerated/Src/stm32f4xx_hal_msp.c \
    ../MDK/RTE/Device/STM32F407ZETx/STCubeGenerated/Src/stm32f4xx_it.c

INCLUDEPATH += ../../../lib/inc


include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ../MDK/RTE/RTE_Components.h \
    ../MDK/RTE/Device/STM32F407ZETx/MX_Device.h \
    ../MDK/RTE/Device/STM32F407ZETx/STCubeGenerated/Inc/stm32f4xx_hal_conf.h \
    ../MDK/RTE/Device/STM32F407ZETx/STCubeGenerated/Inc/stm32f4xx_it.h

DISTFILES += \
    ../MDK/RTE/Device/project.script \
    ../MDK/RTE/Device/STM32F407ZETx/startup_stm32f407xx.s \
    ../MDK/RTE/Device/STM32F407ZETx/STCubeGenerated/STCubeGenerated.txt

