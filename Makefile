#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := blink
CFLAGS:=-DUSE_I2C_2V8=1

include $(IDF_PATH)/make/project.mk

