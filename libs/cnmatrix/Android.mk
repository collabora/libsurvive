LOCAL_PATH := $(call my-dir)

ifneq ($(TARGET_SURVIVE_MATH_BACKEND),)
    SURVIVE_MATH_BACKEND := $(TARGET_SURVIVE_MATH_BACKEND)
else
    SURVIVE_MATH_BACKEND := eigen
endif

include $(CLEAR_VARS)
LOCAL_MODULE                := libcnmatrix
LOCAL_SRC_FILES             := src/cn_matrix.c
LOCAL_MODULE_CLASS          := STATIC_LIBRARIES
LOCAL_C_INCLUDES            := $(LOCAL_PATH)/include
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/include
LOCAL_PROPRIETARY_MODULE    := true

LOCAL_CFLAGS := \
    -Wno-error=unused-const-variable \
    -Wno-error=unused-function \
    -Wno-error=unused-parameter

ifeq ($(SURVIVE_MATH_BACKEND),eigen)
    LOCAL_SRC_FILES        += src/eigen/core.cpp src/eigen/gemm.cpp src/eigen/svd.cpp
    LOCAL_CFLAGS           += -DUSE_EIGEN
    LOCAL_HEADER_LIBRARIES += libeigen
else ifeq ($(SURVIVE_MATH_BACKEND),blas)
    # This needs a blas library to link, which is not easily available
    LOCAL_SRC_FILES        += src/cn_matrix.blas.c
else
    $(error Unsupported cnmatrix backend: $(SURVIVE_MATH_BACKEND))
endif

include $(BUILD_STATIC_LIBRARY)
