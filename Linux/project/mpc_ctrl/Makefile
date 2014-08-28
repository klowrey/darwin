###############################################################
#
# Purpose: Makefile for "mpc_ctrl"
# Author.: robotis & uw
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = streamer
#TARGET2 = joints
TARGET_LOG = sys_id_log 

CXX = g++
INCLUDE_DIRS = -I../../include -I../../../Framework/include
CXXFLAGS +=	-DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lrt


#all: darwin.a $(TARGET) $(TARGET2) $(TARGET_LOG)
all: darwin.a $(TARGET) $(TARGET2)

#clean:
#	rm -f *.a *.o $(TARGET) $(TARGET2) $(TARGET_LOG) core *~ *.so *.lo

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): stream_main.o ../../lib/darwin.a
	$(CXX) -o $(TARGET) stream_main.o ../../lib/darwin.a $(LIBS)
	chmod 755 $(TARGET)

$(TARGET2): joints_main.o ../../lib/darwin.a
	$(CXX) -o $(TARGET2) joints_main.o ../../lib/darwin.a $(LIBS)
	chmod 755 $(TARGET2)

#$(TARGET_LOG): log_main.o ../../lib/darwin.a
#	$(CXX) -o $(TARGET_LOG) ll_main.o ../../lib/darwin.a $(LIBS)
#	chmod 755 $(TARGET_LOG)

#$(TARGET_COMPLIANCE): compliance.o ../../lib/darwin.a
#	$(CXX) -o $(TARGET_COMPLIANCE) compliance.o ../../lib/darwin.a $(LIBS)
#	chmod 755 $(TARGET_COMPLIANCE)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/read_write_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
