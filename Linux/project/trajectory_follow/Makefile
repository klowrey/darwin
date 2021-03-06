###############################################################
#
# Purpose: Makefile for "follower"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = follower 

INCLUDE_DIRS = -I../../include -I../../../Framework/include -I../../../SDK/phasespace/include -I/usr/include/boost -I/usr/include/eigen3

CXX = g++
#CXXFLAGS += -g -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O0 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS) -std=c++11
LFLAGS += -lpthread -ljpeg -lrt -lboost_program_options -L../../../SDK/phasespace/lib -lowlsock
#LFLAGS += -lpthread -ljpeg -lrt -lboost_program_options

OBJECTS =   main.o 

all: $(TARGET) logger test debug_ps

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

test: darwin.a test_main.o 
	$(CXX) $(CFLAGS) test_main.o ../../lib/darwin.a -o test $(LFLAGS)
	chmod 755 test 

ps_logger: darwin.a ps_main.o 
	$(CXX) $(CFLAGS) ps_main.o ../../lib/darwin.a -o ps_logger $(LFLAGS)
	chmod 755 ps_logger 

debug_ps: darwin.a debug_ps.o 
	$(CXX) $(CFLAGS) debug_ps.o ../../lib/darwin.a -o debug_ps $(LFLAGS)
	chmod 755 debug_ps 


# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/follower_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
