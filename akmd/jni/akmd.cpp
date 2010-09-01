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

#include <fcntl.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>

#include "linux-2.6.29/akm8973.h"
#include "linux-2.6.29/bma150.h"

#include "akmd.hpp"

namespace akmd {

Akmd::Akmd(int magnetometer_gain, int temperature_zero)
    : accelerometer(3600), magnetometer(120)
{
    this->analog_gain = magnetometer_gain;
    this->temperature_zero = temperature_zero;

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
    
    fixed_analog_gain = 15;
    calibrate_analog_apply();

    earth = Vector(0, 0, -256);
}

Akmd::~Akmd() {
    SUCCEED(close(akm_fd) == 0);
    SUCCEED(close(bma150_fd) == 0);
}

/****************************************************************************/
/* Analog calibration                                                       */
/****************************************************************************/
char Akmd::akm_analog_offset(int i)
{
    signed char corr = analog_offset[i];
    if (corr < 0) {
        corr = 127 - corr;
    }
    return (char) corr;
}

void Akmd::calibrate_analog_apply()
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
void Akmd::calibrate_magnetometer_analog_helper(float val, int i)
{
    const float ANALOG_MAX = 126.0f;
    const float BOUND_MAX = 240.0f;
    /* The rate of forgetting encountering the minimum or maximum bound.
     * Keeping this fairly large to make it less likely that analog gain
     * gets adjusted by mistake. */
    const float CALIBRATE_DECAY = 0.1f;

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

void Akmd::calibrate_magnetometer_analog(Vector* m)
{
    calibrate_magnetometer_analog_helper(m->x, 0);
    calibrate_magnetometer_analog_helper(m->y, 1);
    calibrate_magnetometer_analog_helper(m->z, 2);    

    /* Apply 16-bit digital gain factor to scale 8->12 bits. */
    *m = m->multiply(digital_gain);
}

void Akmd::calibrate_magnetometer(Vector a, Vector* m)
{
    const int REFRESH = 1;

    calibrate_magnetometer_analog(m);

    magnetometer.update(next_update.tv_sec, a, *m);
    if (magnetometer.fit_time <= next_update.tv_sec - REFRESH) {
        magnetometer.try_fit(next_update.tv_sec);
    }

    /* Correct for scale and offset. */
    *m = m->add(magnetometer.center.multiply(-1));
    *m = m->multiply(magnetometer.scale);
}

void Akmd::calibrate_accelerometer(Vector* a)
{
    const int REFRESH = 60;

    accelerometer_g = accelerometer_g.multiply(0.8f).add(a->multiply(0.2f));

    /* a and g must have about the same length and point to about same
     * direction before I trust the value accumulated to g */
    float al = a->length();
    float gl = accelerometer_g.length();

    /* The alignment of vector lengths and directions must be better than 5 % */
    if (al != 0
        && gl != 0
        && fabsf(al - gl) < 0.04f
        && a->dot(accelerometer_g) / (al * gl) > 0.96f) {

        /* Going to trust this point. */
        accelerometer.update(next_update.tv_sec, *a, accelerometer_g);
        if (accelerometer.fit_time <= next_update.tv_sec - REFRESH) {
            accelerometer.try_fit(next_update.tv_sec);
        }
    }

    *a = a->add(accelerometer.center.multiply(-1));
    *a = a->multiply(accelerometer.scale);
}

/****************************************************************************/
/* Sensor output calculation                                                */
/****************************************************************************/
void Akmd::estimate_earth(Vector a)
{
    /* Smooth acceleration over time to try to establish a less unstable
     * direction towards Earth. Probably the best we can do until gyroscopes.
     */
    earth = earth.multiply(0.9f).add(a.multiply(0.1f));
}

static float rad2deg(float v) {
    return v * (180.0f / (float) M_PI);
}

void Akmd::fill_result_vector(Vector a, Vector m, short temperature, short* out)
{
    calibrate_accelerometer(&a);
    calibrate_magnetometer(a, &m);
    estimate_earth(a);

    /* From g, we need to discover 2 suitable vectors. Cross product
     * is used to establish orthogonal basis in E. */
    Vector ref(-1, 0, 0);
    Vector o1 = earth.cross(ref);
    Vector o2 = earth.cross(o1);
    
    /* Now project magnetic field on components o1 and o2. */
    float o1l = m.dot(o1) * o2.length();
    float o2l = m.dot(o2) * o1.length();

    /* Establish the angle in E */
    out[0] = roundf(180.0f - rad2deg(atan2f(o2l, o1l)));
    /* pitch */
    out[1] = roundf(rad2deg(atan2f(earth.y, -earth.z)));
    /* roll */
    out[2] = roundf(90.0f - rad2deg(acosf(earth.x / earth.length())));
    
    out[3] = -(temperature + temperature_zero);
    out[4] = 3; /* Magnetic accuracy; could be defined as test of quality of sphere fit, but not evaluated right now */
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
void Akmd::sleep_until_next_update()
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

void Akmd::measure()
{
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
    abuf[index] = Vector(bma150_data[0], -bma150_data[1], bma150_data[2]);

    /* Significance and range of values can be extracted from
     * online AK 8973 manual. The kernel driver just passes the data on. */
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data) == 0);
    short temperature = (signed char) akm_data[1];
    mbuf[index] = Vector(127 - (unsigned char) akm_data[2], 127 - (unsigned char) akm_data[3], 127 - (unsigned char) akm_data[4]);

    Vector a = abuf[0].add(abuf[1]).multiply(0.5f);
    Vector m = mbuf[0].add(mbuf[1]).multiply(0.5f);
    index = (index + 1) & 1;

    /* Calculate and set data readable on compass input. */
    short final_data[12];
    fill_result_vector(a, m, temperature, final_data);
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_SET_YPR, &final_data) == 0);
}

void Akmd::wait_start()
{
    /* When open, we enable BMA and wait for close event. */
    int status;
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) == 0);
    SUCCEED(gettimeofday(&next_update, NULL) == 0);
}

void Akmd::start_bma()
{
    char bmode = BMA_MODE_NORMAL;
    SUCCEED(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode) == 0);
}

void Akmd::wait_stop()
{
    /* When open, we enable BMA and wait for close event. */
    int status;
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_CLOSE_STATUS, &status) == 0);
}

void Akmd::stop_bma()
{        
    char bmode = BMA_MODE_SLEEP;
    SUCCEED(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode) == 0);
    SUCCEED(gettimeofday(&next_update, NULL) == 0);
}

}
