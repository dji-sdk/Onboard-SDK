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

SCRIPTSRCS = cmdCamera.cpp \
             cmdCoreAPI.cpp \
             cmdFlight.cpp \
             cmdFollow.cpp \
             cmdHotPoint.cpp \
             cmdIO.cpp \
             cmdSettings.cpp \
             cmdVirtualRC.cpp \
             cmdWayPoint.cpp \
             DJI_Interpreter.cpp \
             DJI_Script.cpp

SRCS = main.cpp \
       conboardsdktask.cpp \
       APIThread.cpp \
       DJIHardDriverManifold.cpp \
       $(LIBSRCS) \
       $(SCRIPTSRCS)

OBJS = $(SRCS:.cpp=.o)

TARGET = onboardSDK

LIBDIR = ../../lib
DJISCRIPTDIR = DJIscript

#VPATH = $(LIBDIR)/inc:$(LIBDIR)/src:$(DJISCRIPTDIR)/inc:$(DJISCRIPTDIR)/src

CC = g++
INC = -I$(LIBDIR)/inc -I$(DJISCRIPTDIR)/inc -Iinc
CXXFLAGS = --std=c++11 $(INC) -pthread

bin/$(TARGET) : $(addprefix objs/, $(OBJS))
	$(CC) -o $@ $(CXXFLAGS) $^

objs/main.o : src/main.cpp 
	$(CC) -o $@ -c $< $(CXXFLAGS)

objs/%.o : src/%.cpp inc/%.h
	$(CC) -o $@ -c $< $(CXXFLAGS)

objs/%.o : $(LIBDIR)/src/%.cpp $(LIBDIR)/inc/%.h
	$(CC) $(CXXFLAGS) -o $@ -c $<

objs/%.o : $(DJISCRIPTDIR)/src/%.cpp $(DJISCRIPTDIR)/inc/%.h
	$(CC) $(CXXFLAGS) $(INC) -o $@ -c $<

clean :
	rm -f objs/* bin/*
