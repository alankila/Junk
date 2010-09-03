#pragma once

#include <android/log.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>

#define SUCCEED(...) if (! (__VA_ARGS__)) { \
LOGI("%s:%d expression '%s' failed: %s", __FILE__, __LINE__, #__VA_ARGS__, strerror(errno)); \
exit(1); \
}

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "akmd.free", __VA_ARGS__)

static float rad2deg(float v) {
    return v * (180.0f / (float) M_PI);
}
