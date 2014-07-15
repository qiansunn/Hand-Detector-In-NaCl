THIS_MAKEFILE := $(abspath $(lastword $(MAKEFILE_LIST)))
NACL_SDK_ROOT ?= $(abspath $(dir $(THIS_MAKEFILE))../..)

# Project Build flags
WARNINGS := -Wno-long-long -Wall -Wswitch-enum -pedantic

#
# Compute tool paths
#
GETOS := python $(NACL_SDK_ROOT)/tools/getos.py
OSHELPERS = python $(NACL_SDK_ROOT)/tools/oshelpers.py
OSNAME := $(shell $(GETOS))
RM := rm

CXXFLAGS := -I$(NACL_SDK_ROOT)/include -I$(NACL_SDK_ROOT)/toolchain/linux_x86_newlib/x86_64-nacl/usr/include
LDFLAGS := -L$(NACL_SDK_ROOT)/lib/newlib_x86_64/Debug -lppapi_cpp -lppapi -lpthread -lopencv_objdetect -lopencv_calib3d -lopencv_features2d -lopencv_imgproc -lopencv_core -lopencv_contrib -lopencv_flann -lopencv_highgui -lz -lnacl_io -lpng
NACL_CXX := $(abspath $(NACL_SDK_ROOT)/toolchain/linux_x86_newlib/bin/x86_64-nacl-g++)

PNACL_TC_PATH := $(abspath $(NACL_SDK_ROOT)/toolchain/$(OSNAME)_pnacl)
PNACL_CXX := $(PNACL_TC_PATH)/bin/pnacl-clang++
PNACL_FINALIZE := $(PNACL_TC_PATH)/bin/pnacl-finalize
PCXXFLAGS := -I$(NACL_SDK_ROOT)/include -I$(NACL_SKD_ROOT)/toolchain/linux_pnacl/usr/include
PLDFLAGS := -L$(NACL_SDK_ROOT)/lib/pnacl/Debug -lppapi_cpp -lppapi -lpthread -lopencv_objdetect -lopencv_calib3d -lopencv_features2d -lopencv_imgproc -lopencv_core -lopencv_contrib -lopencv_flann -lopencv_highgui -lz -lnacl_io -lpng


# Declare the ALL target first, to make the 'all' target the default build
all: main.nexe

clean:
	$(RM) main.nexe

main.nexe: main.cc hand_detector_model.cc
	$(NACL_CXX) -g -o $@ $^ $(WARNINGS) $(CXXFLAGS) $(LDFLAGS)

#main.bc: main.cc hand_detector_model.cc
#	$(PNACL_CXX) -o $@ $^ -O2 $(PCXXFLAGS) $(PLDFLAGS)

#main.pexe: main.bc
#	$(PNACL_FINALIZE) -o $@ $<
	  

#
# Makefile target to run the SDK's simple HTTP server and serve this example.
#
HTTPD_PY := python $(NACL_SDK_ROOT)/tools/httpd.py

.PHONY: serve
serve: all
	$(HTTPD_PY) -C $(CURDIR)
