TARGET = ../output/DJI_Onboard_API_Cmdline_Test

OBJECTS = main.o DJI_Pro_App.o DJI_Pro_Hw.o DJI_Pro_Link.o DJI_Pro_Test.o DJI_Pro_Codec.o

CFLAGS = -Wall -O3 -Isrc/
LDFLAGS = -lpthread

CXX = g++

$(TARGET) : $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
main.o : ../src/main.cpp
	$(CXX) $(CFLAGS) -c ../src/main.cpp

DJI_Pro_App.o : ../src/DJI_Pro_App.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_App.cpp
	
DJI_Pro_Hw.o : ../src/DJI_Pro_Hw.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Hw.cpp

DJI_Pro_Link.o : ../src/DJI_Pro_Link.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Link.cpp

DJI_Pro_Test.o : ../src/DJI_Pro_Test.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Test.cpp
	
DJI_Pro_Codec.o : ../src/DJI_Pro_Codec.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Codec.cpp
	
clean:
	-rm *.o
	-rm $(TARGET)
