LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm
LOCAL_SRC_FILES := akmd.cpp calibrator.cpp main.cpp matrix.cpp vector.cpp
LOCAL_LDLIBS := -llog
LOCAL_MODULE := akmd.free

include $(BUILD_EXECUTABLE)
