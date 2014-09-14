###############################################################
#
# Purpose: Makefile for "read_write"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = phase_accel

CXX = g++
INCLUDE_DIRS = -I../../include -I../../../Framework/include 
LINK_DIRS = 
CXXFLAGS +=	-O2 -DLINUX -Wall -fmessage-length=0 -march=i686 $(INCLUDE_DIRS) 
LIBS += -lpthread -lrt -lowlsock

OBJS =	main.o

all: darwin.a $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): $(OBJS) ../../lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../lib/darwin.a $(LINK_DIRS) $(LIBS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/read_write_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
