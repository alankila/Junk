/*
 * Free, open-source replacement of the closed source akmd driver written
 * by AKM. This program is the user-space counterpart of akm8973 and bma150
 * sensors found on various Android phones.
 *
 * Copyright Antti S. Lankila, 2010, licensed under the Apache License.
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

#include <linux-2.6.29/akm8973.h>
#include <linux-2.6.29/bma150.h>

#define AKM_NAME "/dev/akm8973_daemon"
#define BMA150_NAME "/dev/bma150"

#define SUCCEED(...)\
    do { if (! (__VA_ARGS__)) {                                            \
        LOGI(__FILE__ " ioctl on line %d: %s", __LINE__, strerror(errno)); \
        exit(1);                                                           \
    } } while (0)

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "akmd.free", __VA_ARGS__)

/* Point Cloud Resolution */
#define PCR 32           /* record this number of vectors around the device */
#define PCR_USE_TIME 100 /* time the vector is good for (seconds) */

typedef struct {
    float x, y, z;
    int time;
} point_t;

static point_t point_cloud[PCR];

static struct timeval current_time;
static struct timeval next_update;

static int akm_fd, bma150_fd;

/* Accelerometer centering error. */
static float accelerometer_offset[3];
/* Temperature is -(value-zero). */
static char temperature_zero = 0;
/* The analog offset */
static int analog_offset[3];
/* The user requested analog gain */
static int analog_gain[3];
/* The actual gain used on hardware */
static int fixed_analog_gain = 15;
/* Digital gain to compensate for analog setting. */
static float digital_gain[3];

/****************************************************************************/
/* Some vector utilities                                                    */
/****************************************************************************/
static void cross_product(float *a, float *b, float *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

static float dot(float *a, float *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static float length(float *a)
{
    return sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

/****************************************************************************/
/* Quaternion arithmetics                                                   */
/****************************************************************************/
typedef struct {
    float w, x, y, z;
} quaternion_t;

static void quat(float a, float *v, quaternion_t *q)
{
    a *= 0.5f;
    float sina = sinf(a);
    float cosa = cosf(a);

    q->w = cosa;
    q->x = v[0] * sina;
    q->y = v[1] * sina;
    q->z = v[2] * sina;
}

static void quat_mul(quaternion_t *a, quaternion_t *b, quaternion_t *c)
{
    c->w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
    c->x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    c->y = a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x;
    c->z = a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w;
}

/****************************************************************************/
/* Analog calibration                                                       */
/****************************************************************************/
static void recalculate_digital_gain()
{
    int i;
    for (i = 0; i < 3; i ++) {
        /* 0.4 dB per step */
        digital_gain[i] = powf(10.0f, (analog_gain[i] - fixed_analog_gain) * 0.4f / 20.0f) * 16.0f;
    }
}

static char akm_analog_offset(int i)
{
    /* AKM specification says that values 0 .. 127 are monotonously
     * decreasing corrections, and values 128 .. 255 are
     * monotonously increasing corrections and that 0 and 128
     * are near each other. */
    signed int corr = analog_offset[i];
    if (corr < 0) {
        corr = 127 - corr;
    }
    return corr;
}

static void calibrate_analog_apply()
{
    char params[6] = {
        akm_analog_offset(0), akm_analog_offset(1), akm_analog_offset(2),
        fixed_analog_gain, fixed_analog_gain, fixed_analog_gain,
    };

    int i;
    for (i = 0; i < 6; i ++) {
        char rwbuf[5] = { 2, 0xe1+i, params[i] };
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_WRITE, &rwbuf) == 0);
    }
}

/****************************************************************************/
/* Digital calibration                                                      */
/****************************************************************************/
static void calibrate_magnetometer_analog(float *m)
{
    int i;
    const float ANALOG_MAX = 120.0f;
    /* The rate of forgetting encountering the minimum or maximum bound.
     * Keeping this fairly large to make it less likely that analog gain
     * gets adjusted by mistake. */
    const float CALIBRATE_DECAY = 0.1f;
    static float rc_min[3];
    static float rc_max[3];

    for (i = 0; i < 3; i ++) {
        /* Autoadjust analog parameters */
        if (m[i] > ANALOG_MAX || m[i] < -ANALOG_MAX) {
            analog_offset[i] += m[i] > ANALOG_MAX ? -1 : 1;
            LOGI("Adjusting analog axis %d to %d because of value %f", i, analog_offset[i], m[i]);

            /* The other axes are OK */
            rc_min[i] = 0;
            rc_max[i] = 0;

            calibrate_analog_apply();
            continue;
        }

        /* If recorded digital bounds get close to completely used,
         * we risk having to constantly adjust the analog gain. We
         * should be able to detect this happening as user rotates the
         * device. */
        if (rc_max[i] - rc_min[i] > ANALOG_MAX * 2 * digital_gain[i]
            && fixed_analog_gain > 0) {
            fixed_analog_gain -= 1;
            LOGI("Adjusting analog gain to %d", fixed_analog_gain);
    
            /* Everything will change. Trash state and return. */
            for (i = 0; i < 3; i ++) {
                rc_min[i] = 0;
                rc_max[i] = 0;
            }
            
            calibrate_analog_apply();
            recalculate_digital_gain();
            break;
        }

        /* Apply 16-bit digital gain factor to scale 8->12 bits. */
        m[i] *= digital_gain[i];

        rc_min[i] += CALIBRATE_DECAY;
        rc_max[i] -= CALIBRATE_DECAY;

        /* minimum value seen */
        if (rc_min[i] > m[i]) {
            rc_min[i] = m[i];
        }

        /* maximum value seen */
        if (rc_max[i] < m[i]) {
            rc_max[i] = m[i];
        }
    }
}

static void calibrate_magnetometer_update(float *a, float *m)
{
    /* Record current sample at point cloud using the rough estimate to
     * determine which bin to put the precise measurement in. */
    /* keep top 2 bits of normalized rm, and turn it into index */
    float len = length(a);
    float x = a[0] / len;
    float y = a[1] / len;
    float z = a[2]; /* NB: length ignored */

    int classify(float v) {
        if (v < -0.5f) {
            return 0;
        }
        if (v < 0.0f) {
            return 1;
        }
        if (v < 0.5f) {
            return 2;
        }
        return 3;
    }
    /* 3rd vector is not independent because we are normalized. It can
     * point above or below the xy plane, though, so total of 2+2+1 bits. */
    int idx = classify(x) << 3 | classify(y) << 1 | classify(z) >> 1;

    point_cloud[idx].x = m[0];
    point_cloud[idx].y = m[1];
    point_cloud[idx].z = m[2];
    point_cloud[idx].time = next_update.tv_sec;
}

static float calibrate_magnetometer_eval(float x, float y, float z, float r)
{
    int i;

    float error = 0;
    int n = 0;
    for (i = 0; i < PCR; i ++) {
        if (point_cloud[i].time < next_update.tv_sec - PCR_USE_TIME) {
            continue;
        }

        float dx = point_cloud[i].x - x;
        float dy = point_cloud[i].y - y;
        float dz = point_cloud[i].z - z;

        float d = sqrtf(dx * dx + dy * dy + dz * dz) - r;
        error += d * d;
        n += 1;
    }

    /* Less than 1/4th of bins filled with recent data? */
    if (n < PCR/4) {
        return -1.0f;
    }

    return sqrtf(error / n);
}

static float calibrate_magnetometer_fit(float *fc)
{
    /* Region to use for derivate estimation, 16 = 1 uT.
     * Because the sensor is a bit noisy, it's probably good idea to
     * be a bit fuzzy about the derivate estimation. */
    const float d = 16;
    /* Stepping speed. This should be as large as possible that still
     * seems to converge rapidly. */
    const float step = 16;

    /* This is a bit lame; should estimate derivate with an offset
     * properly calculated for each axis... */
    float x = fc[0] - d/2;
    float y = fc[1] - d/2;
    float z = fc[2] - d/2;
    float r = fc[3] - d/2;

    float error = calibrate_magnetometer_eval(x, y, z, r);
    if (error < 0) {
        return error;
    }

    float dx = calibrate_magnetometer_eval(x+d, y, z, r) - error;
    float dy = calibrate_magnetometer_eval(x, y+d, z, r) - error;
    float dz = calibrate_magnetometer_eval(x, y, z+d, r) - error;
    float dr = calibrate_magnetometer_eval(x, y, z, r+d) - error;

    /* Steepest descent */
    fc[0] -= dx * (step / d);
    fc[1] -= dy * (step / d);
    fc[2] -= dz * (step / d);
    fc[3] -= dr * (step / d);

    return error;
}

static int calibrate_magnetometer(float *a, float *m)
{
    int i;

    static float fine_calibration[4];
    static float error = 99999.0f;

    calibrate_magnetometer_analog(m);

    /* Use sphere fitting algorithm to find optimum surface that fits
     * the known magnetic vectors interpreted as a point cloud. Theory
     * suggests the shape of this data is an ellipsoid, ideally a sphere.
     */
    calibrate_magnetometer_update(a, m);
    float fit_error = calibrate_magnetometer_fit(fine_calibration);
    /* Assume old fit is still good until we can refresh it. */
    if (fit_error >= 0) {
        error = fit_error;
    }
    //LOGI("Spherical fit error: %f", error);

    /* Adjust magnetic field to calibrated center.
     * TBD: add axis gain correction. */
    for (i = 0; i < 3; i ++) {
        m[i] -= fine_calibration[i];
    }

    /* 2 uT average error */
    if (error <= 32.0f) {
        return 3;
    }
    /* 4 uT average error */
    if (error <= 64.0f) {
        return 2;
    }
    /* 8 uT average error */
    if (error <= 128.0f) {
        return 1;
    }
    /* Calibration completely whacked. */
    return 0;
}

static void calibrate_accelerometer(float *a)
{
    /* TBD; needs spherical fit parameters supplied from calibration software */
    int i;
    for (i = 0; i < 3; i ++) {
        a[i] += accelerometer_offset[i];
    }
}

/****************************************************************************/
/* Sensor output calculation                                                */
/****************************************************************************/
static void estimate_earth(float *a, float *m, float *g)
{
    int i;
    static float mh[3];
    static float mrh[3];

    /* This quaternion represents the shortest possible rotation
     * between mh and a. */
    quaternion_t q;
    {  
        float a_cross_mh[3];
        cross_product(a, mh, a_cross_mh);
        float a_cross_mh_l = length(a_cross_mh);

        float a_l = length(a);
        float mh_l = length(mh);
        if (a_l == 0 || mh_l == 0 || a_cross_mh_l == 0) {
            q.w = 0;
            q.x = 1;
            q.y = 0;
            q.z = 0;
        } else {
            float a_dot_mh = dot(a, mh) / (a_l * mh_l);

            a_cross_mh[0] /= a_cross_mh_l;
            a_cross_mh[1] /= a_cross_mh_l;
            a_cross_mh[2] /= a_cross_mh_l;

            quat(acosf(a_dot_mh), a_cross_mh, &q);
        }
    }

    /* Now rotate m on top of a:
     * m' = q * m * q^-1 */
    quaternion_t qm;
    qm.w = 0;
    qm.x = m[0];
    qm.y = m[1];
    qm.z = m[2];

    quaternion_t tmp;
    quat_mul(&q, &qm, &tmp);
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    quat_mul(&tmp, &q, &qm);

    /* Turn the quaternion back to vector, compare with previous, adjust g */
    float mr[3];
    mr[0] = qm.x;
    mr[1] = qm.y;
    mr[2] = qm.z;

    float mr_l = length(mr);
    if (mr_l != 0) {
        mr[0] /= mr_l;
        mr[1] /= mr_l;
        mr[2] /= mr_l;
    }
    float g_l = length(g);

    for (i = 0; i < 3; i ++) {
        /* Slowly correct gravity towards general acceleration direction */
        g[i] = g[i] * 0.99f + a[i] * 0.01f;
        /* Rapidly adjust based on change in magnetic vector */
        g[i] += (mr[i] - mrh[i]) * g_l;
 
        mrh[i] = mr[i];
        mh[i] = m[i];
    }
}

static void build_result_vector(float *a, short temperature, float *m, short *out)
{
    static float g[3];

    calibrate_accelerometer(a);
    int magnetic_quality = calibrate_magnetometer(a, m);
    estimate_earth(a, m, g);

    /* From g, we need to discover 2 suitable vectors. Cross product
     * is used to establish orthogonal basis in E. */
    float o1[3];
    float ref[3] = { -1, 0, 0 };
    cross_product(g, ref, o1);
    float o2[3];
    cross_product(g, o1, o2);
    
    /* Now project magnetic field on components o1 and o2. */
    float o1l = dot(m, o1) * length(o2);
    float o2l = dot(m, o2) * length(o1);

    int rad2deg(float v) {
        return v * (180.0f / (float) M_PI);
    }

    /* Establish the angle in E */
    out[0] = roundf(180.0f - rad2deg(atan2f(o2l, o1l)));
    /* pitch */
    out[1] = roundf(rad2deg(atan2f(g[1], -g[2])));
    /* roll */
    out[2] = roundf(90.0f - rad2deg(acosf(g[0] / length(g))));
    
    out[3] = temperature;
    out[4] = magnetic_quality; /* Magnetic accuracy; result of sphere fit. */
    out[5] = 3; /* BMA150 accuracy; no idea how to determine. */

    // Android wants 720 = 1G, Device has 256 = 1G. */
    out[6] = roundf(a[0] * (720.0f/256.0f));
    out[7] = roundf(a[2] * (720.0f/256.0f));
    out[8] = roundf(a[1] * (-720.0f/256.0f));

    out[9] = roundf(m[0]);
    out[10] = roundf(m[1]);
    out[11] = roundf(-m[2]);
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
            LOGI("missed a beat (CPU/sensors too slow)");
        }
        return;
    }

    struct timespec interval = {
        .tv_sec = sleep_time / 1000,
        .tv_nsec = 1000000 * (sleep_time % 1000),
    };
    SUCCEED(nanosleep(&interval, NULL) == 0);
}

static void readLoop()
{
    int i;
    static int abuf[2][3];
    static int mbuf[2][3];
    static int index = 0;

    /* This ioctl sleeps if the control channel isn't open, so we can use
     * this to pause ourselves. Another thread shuts down the BMA150 chip.
     * AKM 8973 will sleep when not being measured anyway. */
    int status;
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) == 0);

    /* Measuring puts readable state to 0. It is going to take
     * some time before the values are ready. Not using SET_MODE
     * because it contains mdelay(1) which makes measurements spin CPU! */
    char akm_data[5] = { 2, AKECS_REG_MS1, AKECS_MODE_MEASURE };
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_WRITE, &akm_data) == 0);

    /* BMA150 is constantly measuring and filtering, so it never sleeps.
     * The ioctl in truth returns only 3 values, but buffer in kernel is
     * defined as 8 shorts long. */
    short bma150_data[8];
    SUCCEED(ioctl(bma150_fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data) == 0);
    abuf[index][0] = bma150_data[0];
    abuf[index][1] = -bma150_data[1];
    abuf[index][2] = bma150_data[2];

    /* Significance and range of values can be extracted from
     * online AK 8973 manual. The kernel driver just passes the data on. */
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data) == 0);
    short temperature = (signed char) -(akm_data[1] + temperature_zero);
    mbuf[index][0] = 127 - (unsigned char) akm_data[2];
    mbuf[index][1] = 127 - (unsigned char) akm_data[3];
    mbuf[index][2] = 127 - (unsigned char) akm_data[4];

    float a[3];
    float m[3];
    for (i = 0; i < 3; i ++) {
        a[i] = 0.5f * (abuf[0][i] + abuf[1][i]);
        m[i] = 0.5f * (mbuf[0][i] + mbuf[1][i]);
    }
    index = (index + 1) & 1;

    /* Calculate and set data readable on compass input. */
    short final_data[12];
    build_result_vector(a, temperature, m, final_data);
    SUCCEED(ioctl(akm_fd, ECS_IOCTL_SET_YPR, &final_data) == 0);

    sleep_until_next_update();
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

void *aot_tracking_thread(void *arg)
{
    while (1) {
        int status;
        char bmode;

        /* When open, we enable BMA and wait for close event. */
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) == 0);
        bmode = BMA_MODE_NORMAL;
        SUCCEED(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode) == 0);

        /* When closed, we disable BMA and wait for open event. */
        SUCCEED(ioctl(akm_fd, ECS_IOCTL_GET_CLOSE_STATUS, &status) == 0);
        bmode = BMA_MODE_SLEEP;
        SUCCEED(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode) == 0);
    }
}

int main(int argc, char **argv)
{
    int i;
   
    if (argc != 8) {
        printf("Usage: akmd <ax> <ay> <az> <gx> <gy> <gz> <tz>\n");
        printf("\n");
        printf("ax, ay, az = accelerometer offset (g)\n");
        printf("gx, gy, gz = magnetometer gain (0.4 dB)\n");
        printf("tz         = temperature zero offset (C)\n");
        printf("\n");
        printf("Per-axis accelerometer offset needs to be calibrated per device.\n");
        printf("Per-axis magnetic gain needs to be calibrated per device type.\n");
        printf("Temperature offset needs to be calibrated per device type.\n");
        _exit(1);
    }

    /* args 1 .. 3, 4 .. 6 */
    for (i = 0; i < 3; i ++) {
        accelerometer_offset[i] = atof(argv[1+i]) * 256.0f;
        analog_gain[i] = atoi(argv[4+i]);
    }
    temperature_zero = atoi(argv[7]);
 
    open_fds();
    calibrate_analog_apply();
    recalculate_digital_gain();
    SUCCEED(gettimeofday(&next_update, NULL) == 0);

    pthread_t thread_id;
    SUCCEED(pthread_create(&thread_id, NULL, aot_tracking_thread, NULL) == 0);
    while (1) {
        readLoop();
    }
}
