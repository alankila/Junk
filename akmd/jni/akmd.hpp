#pragma once

#include <android/log.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "calibrator.hpp"
#include "vector.hpp"

#define AKM_NAME "/dev/akm8973_daemon"
#define BMA150_NAME "/dev/bma150"

#define SUCCEED(...) if (! (__VA_ARGS__)) { \
LOGI("%s:%d expression '%s' failed: %s", __FILE__, __LINE__, #__VA_ARGS__, strerror(errno)); \
exit(1); \
}

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "akmd.free", __VA_ARGS__)

namespace akmd {

class Akmd {
    private:

    /* Temperature is -(value-zero). */
    char temperature_zero;
    /* The analog offset */
    signed char analog_offset[3];
    /* The user requested analog gain */
    int analog_gain;
    /* The actual gain used on hardware */
    int fixed_analog_gain;
    /* Digital gain to compensate for analog setting. */
    float digital_gain;

    /* Open file descriptors */
    int akm_fd;
    int bma150_fd;

    /* Desired start point of next measurement interval. */
    struct timeval next_update;

    /* Magnetic calibration variables */
    Calibrator magnetometer;
    float rc_min[3];
    float rc_max[3];
  
    /* Accelerometer calibration variables */  
    Calibrator accelerometer;
    Vector accelerometer_g;
    
    /* Direction to Earth center */
    Vector earth;
    
    /* Measurement variables */
    int index;
    Vector abuf[2];
    Vector mbuf[2];

    char akm_analog_offset(int i);
    void calibrate_analog_apply();
    void calibrate_magnetometer_analog_helper(float v, int i);
    void calibrate_magnetometer_analog(Vector *m);
    void calibrate_magnetometer(Vector a, Vector *m);
    void calibrate_accelerometer(Vector *a);
    void estimate_earth(Vector a);
    void fill_result_vector(Vector a, Vector m, short temperature, short* out);

    public:
    Akmd(int magnetic, int temperature);
    ~Akmd();
    void measure();
    void sleep_until_next_update();

    void wait_start();
    void start_bma();
    void wait_stop();
    void stop_bma();
};

}
