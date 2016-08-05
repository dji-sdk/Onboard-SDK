LIBSRCS = DJI_API.cpp \
          DJI_App.cpp \
          DJI_Camera.cpp \
          DJI_Codec.cpp \
          DJI_Flight.cpp \
          DJI_Follow.cpp \
          DJI_HardDriver.cpp \
          DJI_HotPoint.cpp \
          DJI_Link.cpp \
          DJI_Memory.cpp \
          DJI_Mission.cpp \
          DJI_VirtualRC.cpp \
          DJI_WayPoint.cpp

SRCS = main.cpp \
       LinuxThread.cpp \
       LinuxSerialDevice.cpp \
       LinuxSetup.cpp \
       LinuxCleanup.cpp \
       LinuxMobile.cpp \
       LinuxFlight.cpp \
       LinuxWaypoint.cpp\
       LinuxInteractive.cpp\
       ReadUserConfig.cpp\
       $(LIBSRCS)

OBJS = $(SRCS:.cpp=.o)

TARGET = onboardSDK

LIBDIR = ../../lib

CC = g++
INC = -I$(LIBDIR)/inc -Iinc
CXXFLAGS = --std=c++11 $(INC) -pthread -lrt

VPATH = ../../platform:src:inc:$(LIBDIR)/src:$(LIBDIR)/inc

bin/$(TARGET) : $(addprefix objs/, $(OBJS))
	$(CC) -o $@ $(CXXFLAGS) $^

objs/main.o : src/main.cpp 
	$(CC) -o $@ -c $< $(CXXFLAGS)

objs/%.o : %.cpp %.h 
	$(CC) -o $@ -c $< $(CXXFLAGS)

clean :
	rm -f objs/* bin/*
