#!/bin/make

# COMMANDS AND FLAGS

# g++ setup
CPP=g++
CPPFLAGS=-std=c++20 -Dlinux -Iinclude -L. -I/usr/include/ni -I/usr/include/nite -O2 -pthread -latomic -lOpenNI -march=native -flto=auto -pedantic

# ar setup
AR=ar
ARFLAGS=-rc

# object files
OBJ=src/watergun/tracker.o



# USEFUL TARGETS

# all
#
# make libraries and binary
all: test

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