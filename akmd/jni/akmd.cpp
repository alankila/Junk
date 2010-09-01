/*
 * Free, open-source replacement of the closed source akmd driver written
 * by AKM. This program is the user-space counterpart of akm8973 and bma150
 * sensors found on various Android phones.
 *
 * Copyright Antti S. Lankila, 2010, licensed under GPL.
 *
 * The device node to read data from is called akm8973_daemon. The control
 * device node is akm8973_aot. The libsensors from android talks to
 * akm8973_aot. The akmd samples the chip data and performs the analysis.
 * The measuring is inherently a slow process, and therefore a cached
 * copy of results is periodically updated to the /dev/input node "compass"
 * using an ioctl on the akm8973_daemon.
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <android/log.h>

#include "linux-2.6.29/akm8973.h"
#include "linux-2.6.29/bma150.h"

#include "calibrator.hpp"
#include "matrix.hpp"
#include "vector.hpp"

#define AKM_NAME "/dev/akm8973_daemon"
#define BMA150_NAME "/dev/bma150"

#define SUCCEED(...) if (! (__VA_ARGS__)) { \
LOGI("%s:%d expression '%s' failed: %s", __FILE__, __LINE__, #__VA_ARGS__, strerror(errno)); \
exit(1); \
}

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "akmd.free", __VA_ARGS__)

using namespace akmd;

Calibrator magnetometer(120);
Calibrator acceleration(3600);

static struct timeval next_update;

static int akm_fd, bma150_fd;

/* Temperature is -(value-zero). */
static char temperature_zero;
/* The analog offset */
static signed char analog_offset[3];
/* The user requested analog gain */
static int analog_gain;
/* The actual gain used on hardware */
static int fixed_analog_gain;
/* Digital gain to compensate for analog setting. */
static float digital_gain;

/****************************************************************************/
/* Analog calibration                                                       */
/****************************************************************************/
static char akm_analog_offset(int i)
{
    signed char corr = analog_offset[i];
    if (corr < 0) {
        corr = 127 - corr;
    }
    return (char) corr;
}

static void calibrate_analog_apply()
{
    char params[6] = {
        akm_analog_offset(0), akm_analog_offset(1), akm_analog_offset(2),
        fixed_analog_gain, fixed_analog_gain, fixed_analog_gain,
    };

    for (int i = 0; i < 6; i ++) {
        char rwbuf[5] = { 2, 0xe1+i, params[i] };
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_WRITE, &rwbuf) == 0);
    }
    
    digital_gain = powf(10.0f, (analog_gain - fixed_analog_gain) * 0.4f / 20.0f) * 16.0f;
}

/****************************************************************************/
/* Digital calibration                                                      */
/****************************************************************************/
static void calibrate_magnetometer_analog_helper(float val, int i)
{
    const float ANALOG_MAX = 126.0f;
    const float BOUND_MAX = 240.0f;
    /* The rate of forgetting encountering the minimum or maximum bound.
     * Keeping this fairly large to make it less likely that analog gain
     * gets adjusted by mistake. */
    const float CALIBRATE_DECAY = 0.1f;
    static float rc_min[3];
    static float rc_max[3];

    /* Autoadjust analog parameters */
    if (val > ANALOG_MAX || val < -ANALOG_MAX) {
        analog_offset[i] += val > ANALOG_MAX ? -1 : 1;
        LOGI("Adjusting analog axis %d to %d because of value %f", i, analog_offset[i], val);
        calibrate_analog_apply();

        /* The other axes are OK */
        rc_min[i] = 0;
        rc_max[i] = 0;

        /* Destroy all calibration state, we'll have to start over. */
        magnetometer.reset();

        return;
    }

    /* If recorded digital bounds get close to completely used,
     * we risk having to constantly adjust the analog gain. We
     * should be able to detect this happening as user rotates the
     * device. */
    if (rc_max[i] - rc_min[i] > BOUND_MAX && fixed_analog_gain > 0) {
        fixed_analog_gain -= 1;
        LOGI("Adjusting analog gain to %d", fixed_analog_gain);
        calibrate_analog_apply();

        /* Bounds will change on all axes. */
        for (int j = 0; j < 3; j ++) {
            rc_min[j] = 0;
            rc_max[j] = 0;
        }

        return;
    }

    rc_min[i] += CALIBRATE_DECAY;
    rc_max[i] -= CALIBRATE_DECAY;

    /* minimum value seen */
    if (rc_min[i] > val) {
        rc_min[i] = val;
    }

    /* maximum value seen */
    if (rc_max[i] < val) {
        rc_max[i] = val;
    }
}

static void calibrate_magnetometer_analog(Vector* m)
{
    calibrate_magnetometer_analog_helper(m->x, 0);
    calibrate_magnetometer_analog_helper(m->y, 1);
    calibrate_magnetometer_analog_helper(m->z, 2);    

    /* Apply 16-bit digital gain factor to scale 8->12 bits. */
    *m = m->multiply(digital_gain);
}

static int calibrate_magnetometer(Vector a, Vector* m)
{
    static int last_fit_time;
    static Vector ellipsoid_params[2] = {
        Vector(0, 0, 0),
        Vector(1, 1, 1),
    };

    calibrate_magnetometer_analog(m);

    magnetometer.update(next_update.tv_sec, a, *m);
    if (last_fit_time < next_update.tv_sec - 1) {
        if (magnetometer.try_fit(next_update.tv_sec, ellipsoid_params)) {
            last_fit_time = next_update.tv_sec;
        }
    }

    /* Correct for scale and offset. */
    *m = m->add(ellipsoid_params[0].multiply(-1));
    *m = m->multiply(ellipsoid_params[1]);

    return 3;
}

static void calibrate_accelerometer(Vector* a)
{
    static Vector g;
    static int last_fit_time;
    static Vector ellipsoid_params[2] = {
        Vector(0, 0, 0),
        Vector(1, 1, 1),
    };

    g = g.multiply(0.8f).add(a->multiply(0.2f));

    /* a and g must have about the same length and point to about same
     * direction before I trust the value accumulated to g */
    float al = a->length();
    float gl = g.length();

    /* The alignment of vector lengths and directions must be better than 5 % */
    if (al != 0
        && gl != 0
        && fabsf(al - gl) < 0.04f
        && a->dot(g) / (al * gl) > 0.96f) {

        /* Going to trust this point. */
        acceleration.update(next_update.tv_sec, *a, g);
        if (last_fit_time < next_update.tv_sec - 60) {
            if (acceleration.try_fit(next_update.tv_sec, ellipsoid_params)) {
                last_fit_time = next_update.tv_sec;
            }
        }
    }

    *a = a->add(ellipsoid_params[0].multiply(-1));
    *a = a->multiply(ellipsoid_params[1]);
}

/****************************************************************************/
/* Sensor output calculation                                                */
/****************************************************************************/
static void estimate_earth(Vector a, Vector* g)
{
    /* Smooth acceleration over time to try to establish a less unstable
     * direction towards Earth. Probably the best we can do until gyroscopes.
     */
    *g = g->multiply(0.9f).add(a.multiply(0.1f));
}

static float rad2deg(float v) {
    return v * (180.0f / (float) M_PI);
}

static void build_result_vector(Vector a, short temperature, Vector m, short* out)
{
    static Vector g;

    calibrate_accelerometer(&a);
    int magnetic_quality = calibrate_magnetometer(a, &m);
    estimate_earth(a, &g);

    /* From g, we need to discover 2 suitable vectors. Cross product
     * is used to establish orthogonal basis in E. */
    Vector ref(-1, 0, 0);
    Vector o1 = g.cross(ref);
    Vector o2 = g.cross(o1);
    
    /* Now project magnetic field on components o1 and o2. */
    float o1l = m.dot(o1) * o2.length();
    float o2l = m.dot(o2) * o1.length();

    /* Establish the angle in E */
    out[0] = roundf(180.0f - rad2deg(atan2f(o2l, o1l)));
    /* pitch */
    out[1] = roundf(rad2deg(atan2f(g.y, -g.z)));
    /* roll */
    out[2] = roundf(90.0f - rad2deg(acosf(g.x / g.length())));
    
    out[3] = temperature;
    out[4] = magnetic_quality; /* Magnetic accuracy; result of sphere fit, but not evaluated right now */
    out[5] = 3; /* BMA150 accuracy; no idea how to determine. */

    // Android wants 720 = 1G, Device has 256 = 1G. */
    out[6] = roundf(a.x * (720.0f/256.0f));
    out[7] = roundf(a.z * (720.0f/256.0f));
    out[8] = -roundf(a.y * (720.0f/256.0f));

    out[9] = roundf(m.x);
    out[10] = roundf(m.y);
    out[11] = -roundf(m.z);
}

/****************************************************************************/
/* Raw mechanics of sensor reading                                          */
/****************************************************************************/
void sleep_until_next_update()
{
    unsigned short delay;
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_DELAY, &delay) == 0);

    /* Decide if we want "fast" updates or "slow" updates. GAME and UI
     * are defined as <= 60.
     *
     * We prefer a fixed rate, because accelerometer is optimal at its
     * maximum filtering state, 24 Hz. So we sample at 47 Hz and then
     * average successive samples for both magnetometer and accelerometer.
     */
    if (delay <= 60) {
        delay = 21;
    } else {
        /* divide by 2 to get 2 real samples for the slow modes too. */
        delay /= 2;
    }

    /* Find out how long to sleep so that we achieve true periodic tick. */ 
    struct timeval current_time;
    SUCCEED(gettimeofday(&current_time, NULL) == 0);
    next_update.tv_usec += delay * 1000;
    next_update.tv_sec += next_update.tv_usec / 1000000;
    next_update.tv_usec %= 1000000;

    int sleep_time = (next_update.tv_sec - current_time.tv_sec) * 1000
                    + (next_update.tv_usec - current_time.tv_usec) / 1000;
    if (sleep_time < 0) {
        /* Have we fallen behind by more than one update? Resync.
         * I'm going to be silent about this, because these things happen
         * every now and then, and it isn't horribly destructive. */
        if (sleep_time < -delay) {
            next_update = current_time;
        }
        return;
    }

    struct timespec interval;
    interval.tv_sec = sleep_time / 1000;
    interval.tv_nsec = 1000000 * (sleep_time % 1000);
    SUCCEED(nanosleep(&interval, NULL) == 0);
}

static void* read_loop(void *lock)
{
    static Vector abuf[2];
    static Vector mbuf[2];
    static int index = 0;

    /* Failure to lock this mutex means the main thread is holding it.
     * It releases it when going to sleep. */
    while (pthread_mutex_trylock((pthread_mutex_t*) lock) != 0) {
        /* Measuring puts readable state to 0. It is going to take
         * some time before the values are ready. Not using SET_MODE
         * because it contains mdelay(1) which makes measurements spin CPU! */
        char akm_data[5] = { 2, AKECS_REG_MS1, AKECS_MODE_MEASURE };
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_WRITE, &akm_data) == 0);
    
        /* Sleep for 300 us, which is the measurement interval. */ 
        struct timespec interval;
        interval.tv_sec = 0;
        interval.tv_nsec = 300000;
        SUCCEED(nanosleep(&interval, NULL) == 0);

        /* BMA150 is constantly measuring and filtering, so it never sleeps.
         * The ioctl in truth returns only 3 values, but buffer in kernel is
         * defined as 8 shorts long. */
        short bma150_data[8];
        SUCCEED(ioctl(bma150_fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data) == 0);
        abuf[index].x = bma150_data[0];
        abuf[index].y = -bma150_data[1];
        abuf[index].z = bma150_data[2];

        /* Significance and range of values can be extracted from
         * online AK 8973 manual. The kernel driver just passes the data on. */
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data) == 0);
        short temperature = (signed char) -(akm_data[1] + temperature_zero);
        mbuf[index].x = 127 - (unsigned char) akm_data[2];
        mbuf[index].y = 127 - (unsigned char) akm_data[3];
        mbuf[index].z = 127 - (unsigned char) akm_data[4];

        Vector a = abuf[0].add(abuf[1]).multiply(0.5f);
        Vector m = mbuf[0].add(mbuf[1]).multiply(0.5f);
        index = (index + 1) & 1;

        /* Calculate and set data readable on compass input. */
        short final_data[12];
        build_result_vector(a, temperature, m, final_data);
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_SET_YPR, &final_data) == 0);

        sleep_until_next_update();
    }

    return NULL;
}

static void open_fds()
{
    akm_fd = open(AKM_NAME, O_RDONLY);
    SUCCEED(akm_fd != -1);
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_RESET, NULL) == 0);
    calibrate_analog_apply();

    bma150_fd = open(BMA150_NAME, O_RDONLY);
    SUCCEED(bma150_fd != -1);
    SUCCEED(ioctl(bma150_fd, BMA_IOCTL_INIT, NULL) == 0);
    
    char rwbuf[8] = { 1, RANGE_BWIDTH_REG };
    SUCCEED(ioctl(bma150_fd, BMA_IOCTL_READ, &rwbuf) == 0);
    rwbuf[2] = (rwbuf[1] & 0xf8) | 1; /* 47 Hz sampling */
    rwbuf[0] = 2;
    rwbuf[1] = RANGE_BWIDTH_REG;
    SUCCEED(ioctl(bma150_fd, BMA_IOCTL_WRITE, &rwbuf) == 0);
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: akmd <mg> <tz>\n");
        printf("\n");
        printf("mg = magnetometer gain (0.4 dB)\n");
        printf("tz = temperature zero offset (C)\n");
        printf("\n");
        printf("Both parameters are probably device model specific.\n");
        return 1;
    }

    fixed_analog_gain = 15;
    analog_gain = atoi(argv[1]);
    temperature_zero = atoi(argv[2]);
 
    open_fds();
    calibrate_analog_apply();

    while (true) {
        /* When open, we enable BMA and wait for close event. */
        int status;
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) == 0);
        
        char bmode = BMA_MODE_NORMAL;
        SUCCEED(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode) == 0);
        SUCCEED(gettimeofday(&next_update, NULL) == 0);

        /* Start our read thread */
        pthread_mutex_t read_lock;
        SUCCEED(pthread_mutex_init(&read_lock, NULL) == 0);
        SUCCEED(pthread_mutex_lock(&read_lock) == 0);
        pthread_t thread_id;
        SUCCEED(pthread_create(&thread_id, NULL, read_loop, &read_lock) == 0);

        /* When closed, we disable BMA and wait for open event. */
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_CLOSE_STATUS, &status) == 0);
        /* Signal our read thread to stop. */
        SUCCEED(pthread_mutex_unlock(&read_lock) == 0);
        void *result;
        SUCCEED(pthread_join(thread_id, &result) == 0);
        SUCCEED(pthread_mutex_destroy(&read_lock) == 0);
        
        bmode = BMA_MODE_SLEEP;
        SUCCEED(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode) == 0);
    }
}
