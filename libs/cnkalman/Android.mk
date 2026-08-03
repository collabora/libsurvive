LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE                := libcnkalman
LOCAL_SRC_FILES             := src/iekf.c src/kalman.c src/model.cc src/numerical_diff.c
LOCAL_CPP_EXTENSION         := .cc
LOCAL_CFLAGS                := -Wno-error=unused-parameter -Wno-error=unused-variable
LOCAL_MODULE_CLASS          := STATIC_LIBRARIES
LOCAL_C_INCLUDES            := $(LOCAL_PATH)/include $(LOCAL_PATH)/src
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/include
LOCAL_STATIC_LIBRARIES      := libcnmatrix
LOCAL_PROPRIETARY_MODULE    := true
include $(BUILD_STATIC_LIBRARY)
