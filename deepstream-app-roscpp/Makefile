################################################################################
# Copyright (c) 2019-2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
################################################################################


CUDA_VER?=

ROS_DISTRO:=$(ROS_DISTRO)


# 请手动设置CATKIN_WORKSPACE变量，例如：
# make CATKIN_WORKSPACE=/home/your/catkin_ws


ifeq ($(CUDA_VER),)
  $(error "CUDA_VER is not set")
endif

ifeq ($(ROS_DISTRO),)
  $(error "ROS_DISTRO is not set. Please set the ROS_DISTRO environment variable.")
endif


ifeq ($(CATKIN_WORKSPACE),)
  $(error "CATKIN_WORKSPACE is not set and could not be determined from ROS_PACKAGE_PATH. Please provide CATKIN_WORKSPACE as a make variable.")
endif

ifneq (,$(wildcard $(CATKIN_WORKSPACE)/devel/include))
else
  $(error "CATKIN_WORKSPACE path '$(CATKIN_WORKSPACE)/devel/include' does not exist.")
endif

APP:= deepstream-app-roscpp

TARGET_DEVICE = $(shell gcc -dumpmachine | cut -f1 -d -)

NVDS_VERSION:=6.2

LIB_INSTALL_DIR?=$(CATKIN_WORKSPACE)/devel/lib/deepstream_yolo_ros/
APP_INSTALL_DIR?=$(CATKIN_WORKSPACE)/devel/lib/deepstream_yolo_ros/

ifeq ($(TARGET_DEVICE),aarch64)
  CFLAGS:= -DPLATFORM_TEGRA
endif

SRCS:= $(wildcard src/*.c) $(wildcard src/*.cpp)
SRCS+= $(wildcard apps-common/src/*.c)
SRCS+= $(wildcard apps-common/src/deepstream-yaml/*.cpp)

INCS:= $(wildcard *.h)

PKGS:= gstreamer-1.0 gstreamer-video-1.0 x11 json-glib-1.0 gstreamer-app-1.0

OBJS:= $(SRCS:.c=.o)
OBJS:= $(OBJS:.cpp=.o)

CFLAGS+= -g

CFLAGS+= -I./ -I/opt/nvidia/deepstream/deepstream/sources/apps/apps-common/includes \
		 -I/opt/nvidia/deepstream/deepstream/sources/includes -DDS_VERSION_MINOR=1 -DDS_VERSION_MAJOR=5 \
		 -I /usr/local/cuda-$(CUDA_VER)/include \
		 -I/opt/ros/$(ROS_DISTRO)/include \
		 -I$(CATKIN_WORKSPACE)/devel/include

LIBS:= -L/usr/local/cuda-$(CUDA_VER)/lib64/ -lcudart \
	   -L/opt/ros/$(ROS_DISTRO)/lib \
	   -lroscpp -lrosconsole -lroscpp_serialization -lrostime -lrosconsole_log4cxx -lrosconsole_backend_interface -lxmlrpcpp

LIBS+= -L$(LIB_INSTALL_DIR) -L/opt/nvidia/deepstream/deepstream-$(NVDS_VERSION)/lib/ -lnvdsgst_meta -lnvds_meta -lnvdsgst_helper -lnvdsgst_customhelper \
	  -lnvdsgst_smartrecord -lnvds_utils -lnvds_msgbroker -lm -lyaml-cpp \
	-lcuda -lgstrtspserver-1.0 -ldl -Wl,-rpath,$(LIB_INSTALL_DIR)

CFLAGS+= $(shell pkg-config --cflags $(PKGS))

LIBS+= $(shell pkg-config --libs $(PKGS))

all: $(APP)

%.o: %.c $(INCS) Makefile
	$(CC) -c -o $@ $(CFLAGS) $<

%.o: %.cpp $(INCS) Makefile
	$(CXX) -c -o $@ $(CFLAGS) $<

$(APP): $(OBJS) Makefile
	$(CXX) -o $(APP) $(OBJS) $(LIBS)

install: $(APP)
	mkdir -p $(APP_INSTALL_DIR)
	cp -rv $(APP) $(APP_INSTALL_DIR)

clean:
	rm -rf $(OBJS) $(APP)
