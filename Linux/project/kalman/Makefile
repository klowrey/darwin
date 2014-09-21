# Copyright (c) 2014 Dylan Holmes. All rights reserved.
# 
# Makefile for Kalman filter.

INCLUDES = -I../../include -I../../../Framework/include 
INCLUDES += -I/usr/include/eigen3

CXX = g++
CXXFLAGS = $(INCLUDES) $(LIBS)
#CXXFLAGS +=	-O2 -DLINUX -Wall -fmessage-length=0 -march=i686
LIBS += -lpthread -lrt

.PHONY: all clean

all: darwin.a main

main: main.o ../../lib/darwin.a
	$(CXX) $^ -o $@ $(CXXFLAGS) 

%.o: %.cc
	$(CXX) -c $^ -o $@ $(CXXFLAGS)
	chmod 755 $@

darwin.a:
	make -C ../../build

clean:
	rm *.o

