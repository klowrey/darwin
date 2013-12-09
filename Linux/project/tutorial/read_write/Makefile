###############################################################
#
# Purpose: Makefile for "read_write"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = read_write
TARGET2 = test_read_write
TARGET_LOG = sys_id_log 

CXX = g++
INCLUDE_DIRS = -I../../../include -I../../../../Framework/include
CXXFLAGS +=	-O3 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lrt

OBJS =	main.o


all: darwin.a $(TARGET) $(TARGET2) $(TARGET_LOG)

clean:
	rm -f *.a *.o $(TARGET) $(TARGET2) $(TARGET_LOG) core *~ *.so *.lo

libclean:
	make -C ../../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../../build

$(TARGET): $(OBJS) ../../../lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../../lib/darwin.a $(LIBS)
	chmod 755 $(TARGET)

$(TARGET2): ll_main.o ../../../lib/darwin.a
	$(CXX) -o $(TARGET2) ll_main.o ../../../lib/darwin.a $(LIBS)
	chmod 755 $(TARGET2)

$(TARGET_LOG): log_main.o ../../../lib/darwin.a
	$(CXX) -o $(TARGET_LOG) ll_main.o ../../../lib/darwin.a $(LIBS)
	chmod 755 $(TARGET_LOG)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/read_write_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
