WIC_HOME = /opt/workswell/wic_sdk
EBUS_HOME = /opt/pleora/ebus_sdk/Ubuntu-x86_64
OPENCV_HOME = /home/work/opencv-2.4

CXXFLAGS = -O2 -g -Wall -D_UNIX_ -D_LINUX_ -I/$(WIC_HOME)/include -I/$(EBUS_HOME)/include -std=c++11
CXXFLAGS+=-DGIT_VERSION='"$(shell git describe --always || echo unknown)"'
CXXFLAGS+=-I/$(OPENCV_HOME)/include

LDFLAGS = -L/$(OPENCV_HOME)/lib -L/$(WIC_HOME)/lib -L/$(EBUS_HOME)/lib -Wl,-rpath-link=$(EBUS_HOME)/lib -Wl,-rpath-link=$(EBUS_HOME)/lib/genicam/bin/Linux64_x64

# Use LDFLAGS_EXTRA like below if you compile the program on a system
# with newer libstdc++ and running the resulting binary on the target
# systems fails with:
#
# /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.26' not found

#LDFLAGS_EXTRA = -static-libstdc++

LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lWIC_SDK -ljpeg -lPvBase -lPvDevice -lPvBuffer -lPvGenICam -lPvTransmitter -lPvVirtualDevice -lPvAppUtils -lPvPersistence -lPvSerial -lPvStream -pthread -lboost_system -fsanitize=thread

thermocam-pcb: thermocam-pcb.cpp
	$(CXX) $(CXXFLAGS) -o thermocam-pcb thermocam-pcb.cpp $(LDFLAGS) $(LDFLAGS_EXTRA) $(LIBS)

clean:
	-rm thermocam-pcb
