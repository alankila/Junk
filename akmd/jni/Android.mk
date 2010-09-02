LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm
LOCAL_SRC_FILES := akmd.cpp calibrator.cpp main.cpp matrix.cpp vector.cpp \
    device/akm8973_2_6_29.cpp device/akm8973_temperature_2_6_29.cpp \
    device/akm8973_writer_2_6_29.cpp device/bma150.cpp
LOCAL_LDLIBS := -llog
LOCAL_MODULE := akmd.free

include $(BUILD_EXECUTABLE)
