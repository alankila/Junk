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

#include <math.h>
#include <sys/time.h>
#include <time.h>

#include "Akmd.hpp"

static float rad2deg(float v) {
    return v * (180.0f / (float) M_PI);
}

namespace akmd {

Akmd::Akmd(ChipReader* magnetometer_reader, ChipReader* accelerometer_reader,
    ChipReader* temperature_reader, ChipWriter* result_writer)
    : accelerometer(3600), magnetometer(120), earth(0, 0, -256)
{
    this->magnetometer_reader = magnetometer_reader;
    this->accelerometer_reader = accelerometer_reader;
    this->temperature_reader = temperature_reader;
    this->result_writer = result_writer;
}

Akmd::~Akmd() {
}

void Akmd::calibrate_magnetometer(Vector a, Vector* m)
{
    const int REFRESH = 1;

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

void Akmd::fill_result_vector(Vector a, Vector m, short temperature, short* out)
{
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
    
    out[3] = temperature;
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
    int delay = magnetometer_reader->get_delay();

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
    Vector a = accelerometer_reader->read();
    Vector m = magnetometer_reader->read();
    Vector temp = temperature_reader->read();
    short temperature = (short) temp.x;

    /* Handle calibrations. */    
    calibrate_accelerometer(&a);
    calibrate_magnetometer(a, &m);
    estimate_earth(a);

    /* Calculate and set data readable on compass input. */
    short final_data[12];
    fill_result_vector(a, m, temperature, final_data);
    result_writer->write(final_data);
}

void Akmd::start()
{
    accelerometer_reader->start();
    magnetometer_reader->start();
    temperature_reader->start();
    SUCCEED(gettimeofday(&next_update, NULL) == 0);
}

void Akmd::stop()
{
    accelerometer_reader->stop();
    magnetometer_reader->stop();
    temperature_reader->stop();
}

}
