LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := akmd.c
LOCAL_LDLIBS := -llog
LOCAL_MODULE := akmd.free

include $(BUILD_EXECUTABLE)
