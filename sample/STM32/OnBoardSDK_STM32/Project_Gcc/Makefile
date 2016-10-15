###
# Purpose: to create a bare-metal project with STM32
###
# GNU ARM Embedded Toolchain
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line


BINDIR = .
OSDKROOT = ../../../..
CORELIBROOT = $(OSDKROOT)/osdk-core
DEMOROOT = $(OSDKROOT)/sample/STM32/OnBoardSDK_STM32
STMDRIVER = $(DEMOROOT)/Lib/STM32F4xx_StdPeriph_Driver
CMSISDRIVER = $(DEMOROOT)/Lib/CMSIS
USER_SRC = $(DEMOROOT)/User
BSP_SRC = $(DEMOROOT)/Bsp

###

ASMSOURCES=./startup_stm32f40xx.s

CSOURCES=$(CMSISDRIVER)/src/system_stm32f4xx.c \
         $(STMDRIVER)/src/misc.c \
		 $(STMDRIVER)/src/stm32f4xx_exti.c \
		 $(STMDRIVER)/src/stm32f4xx_gpio.c \
		 $(STMDRIVER)/src/stm32f4xx_rcc.c \
		 $(STMDRIVER)/src/stm32f4xx_syscfg.c \
		 $(STMDRIVER)/src/stm32f4xx_usart.c \
		 $(STMDRIVER)/src/stm32f4xx_tim.c \
		 $(STMDRIVER)/src/stm32f4xx_rtc.c \
		 $(STMDRIVER)/src/stm32f4xx_i2c.c \
		 $(STMDRIVER)/src/stm32f4xx_pwr.c \
		 $(STMDRIVER)/src/stm32f4xx_rng.c \
		 $(STMDRIVER)/src/stm32f4xx_wwdg.c \
		 $(STMDRIVER)/src/stm32f4xx_iwdg.c \
		 ./syscalls.c 

OSDKSOURCES := $(CORELIBROOT)/src/DJI_API.cpp         \
			$(CORELIBROOT)/src/DJI_App.cpp         \
			$(CORELIBROOT)/src/DJI_Camera.cpp      \
			$(CORELIBROOT)/src/DJI_Codec.cpp       \
			$(CORELIBROOT)/src/DJI_Flight.cpp      \
			$(CORELIBROOT)/src/DJI_Follow.cpp      \
			$(CORELIBROOT)/src/DJI_HardDriver.cpp  \
			$(CORELIBROOT)/src/DJI_HotPoint.cpp    \
			$(CORELIBROOT)/src/DJI_Link.cpp        \
			$(CORELIBROOT)/src/DJI_Memory.cpp      \
			$(CORELIBROOT)/src/DJI_Mission.cpp     \
			$(CORELIBROOT)/src/DJI_VirtualRC.cpp   \
			$(CORELIBROOT)/src/DJI_WayPoint.cpp
			#$(CORELIBROOT)/src/DJI_Logging.cpp    \

#OSDKSOURCES := $(wildcard $(CORELIBROOT)/src/*.cpp)
#OSDKSOURCES := $(filter-out $(CORELIBROOT)/src/DJI_Logging.cpp, $(wildcard $(CORELIBROOT)/src/*.cpp))

#MYSOURCES   = $(wildcard $(USER_SRC)/*.cpp) 
#MYSOURCES += $(wildcard $(BSP_SRC)/*.cpp) 

MYSOURCES = $(USER_SRC)/Activate.cpp        \
			$(USER_SRC)/cppforstm32.cpp     \
			$(USER_SRC)/driver.cpp          \
			$(USER_SRC)/HotPoint.cpp        \
			$(USER_SRC)/LocalNavigation.cpp \
			$(USER_SRC)/main.cpp            \
			$(USER_SRC)/MobileCommand.cpp   \
			$(USER_SRC)/Receive.cpp         \
			$(USER_SRC)/VirtualRC.cpp       \
			$(BSP_SRC)/bsp.cpp              \
			$(BSP_SRC)/BspUsart.cpp         \
			$(BSP_SRC)/timer.cpp       

CXXSOURCES = $(OSDKSOURCES) \
             $(MYSOURCES)
	 
		 
INC=$(CORELIBROOT)/inc \
	$(CMSISDRIVER)/inc \
	$(STMDRIVER)/inc \
	$(USER_SRC) \
	$(BSP_SRC)
	

# Find header directories
#INC=$(shell find -L $(INCDIR) -name '*.h' -exec dirname {} \; | uniq)
INCLUDES=$(INC:%=-I%)
# Find libraries
INCLUDES_LIBS=
LINK_LIBS=$(CMSISDRIVER)/libarm_cortexM4lf_math.a
# Create object list
OBJECTS=$(ASMSOURCES:%.s=%.o)
OBJECTS+=$(CSOURCES:%.c=%.o)
OBJECTS+=$(CXXSOURCES:%.cpp=%.o)
# Define output files ELF & IHEX
BINELF=osdk_stm32_gcc.elf
BINHEX=osdk_stm32_gcc.hex

###
# MCU FLAGS
MCFLAGS=-mcpu=cortex-m4 -mthumb -mlittle-endian \
-mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork -ffunction-sections -fdata-sections
# COMPILE FLAGS
DEFS=-DUSE_STDPERIPH_DRIVER -DSTM32F407xx -DSTM32 -D__FPU_PRESENT -DARM_MATH_CM4
CFLAGS=-c $(MCFLAGS) $(DEFS) $(INCLUDES)
CXXFLAGS=-c $(MCFLAGS) $(DEFS) $(INCLUDES)
# LINKER FLAGS
LDSCRIPT=stm32_flash.ld
LDFLAGS =-T$(LDSCRIPT) $(MCFLAGS) --specs=nano.specs  -specs=nosys.specs  -Wl,--gc-sections $(INCLUDES_LIBS) $(LINK_LIBS)

###
# Build Rules
.PHONY:  release  clean


release: $(BINDIR)/$(BINHEX)

$(BINDIR)/$(BINHEX): $(BINDIR)/$(BINELF)
	$(CP) -O ihex $< $@
	@echo "Objcopy from ELF to IHEX complete!\n"

##
# C++ linking is used.
#
# Change
#   $(CXX) $(OBJECTS) $(LDFLAGS) -o $@ to 
#   $(CC) $(OBJECTS) $(LDFLAGS) -o $@ if
#   C linker is required.
$(BINDIR)/$(BINELF): $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	@echo "Linking complete!\n"
	$(SIZE) $(BINDIR)/$(BINELF)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@
	@echo "Compiled "$<"!\n"

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@
	@echo "Compiled "$<"!\n"

%.o: %.s
	$(CC) $(CFLAGS) $< -o $@
	@echo "Assambled "$<"!\n"

clean:
	rm -f $(OBJECTS) $(BINDIR)/$(BINELF) $(BINDIR)/$(BINHEX) $(BINDIR)/output.map

