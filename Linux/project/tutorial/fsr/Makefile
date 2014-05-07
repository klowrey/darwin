###############################################################
#
# Purpose: Makefile for "fsr"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = fsr
TARGET2 = file_data

CXX = g++
INCLUDE_DIRS = -I../../../include -I../../../../Framework/include
CXXFLAGS +=	-DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lrt -ljpeg

OBJS =	main.o


all: darwin.a $(TARGET) $(TARGET2) step stand slack straight walk action

darwin.a:
	make -C ../../../build

$(TARGET): $(OBJS) ../../../lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../../lib/darwin.a $(LIBS)
	
$(TARGET2): file_main.o ../../../lib/darwin.a
	$(CXX) -o $(TARGET2) file_main.o ../../../lib/darwin.a -lpthread -lrt

step: step_main.o ../../../lib/darwin.a
	$(CXX) -o step step_main.o ../../../lib/darwin.a -lpthread -lrt

stand: stand_main.o ../../../lib/darwin.a
	$(CXX) -o stand stand_main.o ../../../lib/darwin.a -lpthread -lrt

slack: stand_main.o ../../../lib/darwin.a
	$(CXX) -o slack stand_main.o ../../../lib/darwin.a -lpthread -lrt

straight: straight_main.o ../../../lib/darwin.a
	$(CXX) -o straight straight_main.o ../../../lib/darwin.a -lpthread -lrt

walk: walk_main.o ../../../lib/darwin.a
	$(CXX) -o walk walk_main.o ../../../lib/darwin.a -lpthread -lrt

action: action_main.o ../../../lib/darwin.a
	$(CXX) -o action action_main.o ../../../lib/darwin.a -lpthread -lrt

clean:
	rm -f $(OBJS) file_main.o step_main.o $(TARGET) $(TARGET2) step stand slack straight walk action

libclean:
	make -C ../../../build clean

distclean: clean libclean

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/tutorial_fsr_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *

