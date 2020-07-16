#!/bin/sh
#Please run it in the gitbash or shell
git config --global core.longpaths true
git clone https://github.com/FreeRTOS/FreeRTOS.git FreeRTOS_10_2_1
cd FreeRTOS_10_2_1
git pull origin
git checkout V10.2.1
cd ..
rm -rf FreeRTOS
cp -r FreeRTOS_10_2_1/FreeRTOS/Source FreeRTOS
cp FreeRTOS_10_2_1/FreeRTOS/Demo/CORTEX_M4F_STM32F407ZG-SK/FreeRTOSConfig.h FreeRTOS/include/
patch -p0 -i FreeRTOSConfig_h_diff.patch
sleep 10