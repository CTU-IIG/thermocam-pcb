WIC_HOME = /opt/workswell/wic_sdk
EBUS_HOME = /opt/pleora/ebus_sdk/Ubuntu-x86_64

CC = g++
CFLAGS = -g -Wall -D_UNIX_ -D_LINUX_ -I/$(WIC_HOME)/include -I/$(EBUS_HOME)/include -std=c++11
CFLAGS+=-DGIT_VERSION='"$(shell git describe --always || echo unknown)"'
SRCS = thermocam-pcb.cpp
PROG = thermocam-pcb

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV) -L/$(WIC_HOME)/lib -L/$(EBUS_HOME)/lib -lWIC_SDK -ljpeg -lPvBase -lPvDevice -lPvBuffer -lPvGenICam -lPvTransmitter -lPvVirtualDevice -lPvAppUtils -lPvPersistence -lPvSerial -lPvStream -pthread

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	-rm thermocam-pcb
