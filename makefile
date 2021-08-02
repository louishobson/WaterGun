#!/bin/make

# COMMANDS AND FLAGS

# g++ setup
CPP=g++
CPPFLAGS=-std=c++20 -Dlinux -Iinclude -L. -I/usr/local/include/OpenNI2 -I/usr/local/include/NiTE2 -O0 -g -pthread -latomic -lOpenNI2 -lNiTE2 -lmraa -march=native -flto=auto -pedantic

# ar setup
AR=ar
ARFLAGS=-rc

# object files
OBJ=src/watergun/tracker.o src/watergun/aimer.o src/watergun/controller.o src/watergun/stepper.o



# USEFUL TARGETS

# all
#
# make libraries and binary
all: test main

# clean
#
# remove all object files and libraries
.PHONY: clean
clean:
	find . -type f -name "*\.o" -delete -print
	find . -type f -name "*\.a" -delete -print
	find . -type f -name "*\.so" -delete -print



# COMPILATION TARGETS

# test
#
# compile test binary
test: $(OBJ) test.o
	$(CPP) $(CPPFLAGS) $(OBJ) test.o -o test

# main
#
# compile main binary
main: $(OBJ) main.o
	$(CPP) $(CPPFLAGS) $(OBJ) main.o -o main

# libwatergun.a
#
# compile into a static library
libwatergun.a: $(OBJ)
	$(AR) $(ARFLAGS) libwatergun.a $(OBJ)

# libwatergun.so
#
# compile into a dynamic library
libwatergun.so: $(OBJ)
	$(CPP) $(OBJ) -shared -o libwatergun.so