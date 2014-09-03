###############################################################
#
# Purpose: Makefile for "follower"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = follower 

INCLUDE_DIRS = -I../../include -I../../../Framework/include -I/usr/include/boost

CXX = g++
CXXFLAGS += -g -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt -lboost_program_options

OBJECTS =   main.o 

all: $(TARGET) logger

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

logger: darwin.a save_main.o 
	$(CXX) $(CFLAGS) save_main.o ../../lib/darwin.a -o logger $(LFLAGS)
	chmod 755 logger 

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/follower_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
