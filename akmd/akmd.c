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
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <utils/Log.h>

#include "akm8973.h"
#include "bma150.h"

#define AKM_NAME "/dev/akm8973_daemon"
#define BMA150_NAME "/dev/bma150"

static int akm_fd, bma150_fd;

typedef enum { READ, SLEEP } state_t;

static char temperature_zero = 0;

/* The actual gain used on hardware */
static int fixed_analog_gain = 15;
/* Digital gain to compensate for analog setting. */
static int digital_gain[3];
/* The user requested analog gain */
static int analog_gain[3];
/* The user requested analog offset */
static int analog_offset[3];

static void calibrate_analog_apply()
{
    char params[6] = {
        analog_offset[0], analog_offset[1], analog_offset[2],
        fixed_analog_gain, fixed_analog_gain, fixed_analog_gain,
    };

    int i;
    for (i = 0; i < 6; i ++) {
        char rwbuf[5] = { 2, 0xe1+i, params[i], 0, 0 };
        if (ioctl(akm_fd, ECS_IOCTL_WRITE, &rwbuf) != 0) {
            perror("Failed to write analog controls\n");
            _exit(3);
        }
    }

    for (i = 0; i < 3; i ++) {
        int g = analog_gain[i];
        digital_gain[i] = powf(10.0f, (g - fixed_analog_gain) * 0.4f / 20.0f) * 65536.0f;
    }
}

static void calibrate_analog(int i, int dir)
{
    analog_offset[i] += dir;
    LOGI("Adjusting analog offset on axis %d to %d", i, analog_offset[i]);
    calibrate_analog_apply();
}

static void calibrate_digital(int *m)
{
    /* Establish the min-max bounds for every measurement seen so far with
     * decay function. */
    static int calibrate_min[3] = {  4096,  4096,  4096 };
    static int calibrate_max[3] = { -4096, -4096, -4096 };
    /* Earth's magnetic field. It's probably a good idea to estimate this
     * slightly low so that extreme values get more rapidly rejected. */
    const int EXPECTED_MAGNETIC_FIELD_UT = 40;
    /* Stop moving bound if it's closer than this value from measurement.
     * 4 follows from 1/16, the 1 from the fact that maximum expected distance
     * is twice the field strength. Is estimate of field's value sufficient?
     * It might be safer to use slightly lower value, and compensate by doing
     * the decay slower. */
    const int BELIEVABLE_BOUND = EXPECTED_MAGNETIC_FIELD_UT << (4 + 1);

    //LOGI("a=(%d %d %d), m=(%d %d %d)", a[0], a[1], a[2], m[0], m[1], m[2]);
    int i;
    for (i = 0; i < 3; i ++) {
        /* Analog clipping handling */
        if (m[i] == 127 || m[i] == -128) {
            calibrate_analog(i, m[i] == 127 ? -1 : 1);
        }

        m[i] = m[i] * digital_gain[i] >> 12;
        /* 1/16 uT */

        /* minimum value seen */
        if (calibrate_min[i] > m[i]) {
            calibrate_min[i] = m[i];
        }
        /* maximum value seen */
        if (calibrate_max[i] < m[i]) {
            calibrate_max[i] = m[i];
        }

        /* gradually move too large-seeming minimum closer to the current
         * measurement point. This compensates for temperature-dependent
         * drifts and recovers after close encounters of magnetic kind. */
        if (m[i] > calibrate_min[i] + BELIEVABLE_BOUND) {
            calibrate_min[i] += 1;
        }
        if (m[i] < calibrate_max[i] - BELIEVABLE_BOUND) {
            calibrate_max[i] -= 1;
        }

        /* Apply current estimate of the midpoint correction adjustment. */
        int correction = (calibrate_max[i] + calibrate_min[i]) >> 1;
        m[i] -= correction;
    }

    /* We should also estimate the analog gain from examining the value
     * range in x, y and z directions, and assume that the magnetic field
     * being measured is the Earth's magnetic field and try to adjust
     * gain such that it gives 45. This could be seen to be cheating.
     * The alternative is to force ROM vendors to supply reasonable gain
     * values. */
}

static void cross_product(int *a, float *b, float *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

static float dot(int *a, float *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static float length_f(float *a)
{
    return sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

static float length_i(int *a)
{
    return sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

static void build_result_vector(int *a, short temperature, int *m, short *out)
{
    calibrate_digital(m);
 
    /*
     * I define yaw in the tangent plane E of the Earth, where direction
     * o2 points towards magnetic North.
     *
     * The device reports an acceleration vector a, which I take to be the
     * normal that defines the tangent space.
     *
     * The device also reports magnetic vector m, which needs to be projected
     * to the tangent space, and then the angle of that vector is to be
     * measured within the tangent space.
     */

    /* From a, we need to discover 2 suitable vectors. Cross product
     * is used to establish orthogonal basis in E. */
    float o1[3];
    float ref[3] = { 0, -1, 0 };
    cross_product(a, ref, o1);
    float o2[3];
    cross_product(a, o1, o2);
    
    /* Now project magnetic field on components o1 and o2. */
    float o1l = dot(m, o1) * length_f(o2);
    float o2l = dot(m, o2) * length_f(o1);

    /* Establish the angle in E */
    out[0] = 180.0f + atan2f(o1l, o2l) / (float) M_PI * 180.0f;
    /* pitch */
    out[1] = atan2f(a[1], -a[2]) / (float) M_PI * 180.0f;
    /* roll */
    out[2] = 90.0f - acosf(a[0] / length_i(a)) / (float) M_PI * 180.0f;
    
    out[3] = temperature;
    /* FIXME: how to establish accuracy? */
    out[4] = 3; // status of mag. sensor (UNRELIABLE, LOW, MEDIUM, HIGH)
    out[5] = 3; // status of acc. sensor (UNRELIABLE, LOW, MEDIUM, HIGH)

    // Android wants 720 = 1G, Device has 256 = 1G. */
    out[6] = (128 + a[0] * 720) >> 8;
    out[7] = (128 + a[2] * 720) >> 8;
    out[8] = (128 + a[1] * -720) >> 8;

    out[9]  =  m[0];
    out[10] =  m[1];
    out[11] = -m[2];
}

static state_t readLoop()
{
    {
        unsigned int status;
        if (ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) != 0) {
            perror("akm8973: Failed to query control channel (aot) open count");
            _exit(10);
        }

        /* Nobody has aot socket open? We'll close the shop... */
        if (status == 0) {
            short mode = AKECS_MODE_POWERDOWN;
            if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, &mode) != 0) {
                perror("akm8973: Failed to SET_MODE=POWERDOWN");
                _exit(11);
            }
            mode = BMA_MODE_SLEEP;
            if (ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &mode) != 0) {
                perror("bma150: Failed to SET_MODE=SLEEP");
                _exit(12);
            }

            return SLEEP;
        }
    }

    /* Measuring puts readable state to 0. It is going to take
     * some time before the values are ready. */
    {
        short mode = AKECS_MODE_MEASURE;
        if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, &mode) != 0) {
            perror("akm8973: Failed to SET_MODE=READ");
            _exit(13);
        }
    }

    int a[3];
    {
        /* Significance and range of values can be extracted from bma150.c. */
        short bma150_data[7]; // note: ioctl says 7 values, kernel driver writes 3
        if (ioctl(bma150_fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data) != 0) {
            perror("bma150: Failed to READ_ACCELERATION");
            _exit(14);
        }
        a[0] = bma150_data[0];
        a[1] = -bma150_data[1];
        a[2] = bma150_data[2];
    }

    /* Significance and range of values can be extracted from
     * online AKM 8973 manual. The kernel driver is dumb. */
    short temperature;
    int m[3];
    {
        char akm_data[5];
        if (ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data) != 0) {
            perror("akm8973: Failed to GETDATA");
            _exit(15);
        }
        temperature = (signed char) -(akm_data[1] + temperature_zero);
        m[0] = 127 - (unsigned char) akm_data[2];
        m[1] = 127 - (unsigned char) akm_data[3];
        m[2] = 127 - (unsigned char) akm_data[4];
    }

    short final_data[12];
    build_result_vector(a, temperature, m, final_data);

    /* Put data to be readable from compass input. */
    if (ioctl(akm_fd, ECS_IOCTL_SET_YPR, &final_data) != 0) {
        perror("bma150: Failed to SET_YPR={akm data}");
        _exit(16);
    }

    /* Get time until next update. */
    unsigned short delay;
    if (ioctl(akm_fd, ECS_IOCTL_GET_DELAY, &delay) != 0) {
         perror("akm8973: Failed to GET_DELAY");
        _exit(17);
    }

    /* We could actually use gettimeofday when we start to
     * achieve truly periodic timer tick. Right now we really
     * sleep for interval + processing time. */
    //delay = 500;
    struct timespec interval = {
        .tv_sec = delay / 1000,
        .tv_nsec = 1000000 * (delay % 1000),
    };
    nanosleep(&interval, NULL);

    return READ;
}

static state_t sleepLoop()
{
    int status;
    short mode;
    struct timespec interval;

    /* Aot has been opened? Resume if so. */
    if (ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) != 0) {
        perror("akm8973: Failed to query control channel (AOT) open count");
        _exit(20);
    }

    if (status != 0) {
        mode = BMA_MODE_NORMAL;
        // akm device is woken by readLoop().
        if (ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &mode) != 0) {
            perror("bma150: Failed to SET_MODE=NORMAL");
            _exit(21);
        }
        return READ;
    }

    interval.tv_sec = 1;
    interval.tv_nsec = 0;
    nanosleep(&interval, NULL);
    return SLEEP;
}

static void open_fds()
{
    akm_fd = open(AKM_NAME, O_RDONLY);
    if (akm_fd == -1) {
        perror("Failed to open " AKM_NAME);
        _exit(1);
    }
    short mode = AKECS_MODE_POWERDOWN;
    if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, &mode) != 0) {
        perror("Failed to put akm8973 to sleep");
        _exit(2);
    }

    bma150_fd = open(BMA150_NAME, O_RDONLY);
    if (bma150_fd == -1) {
        perror("Failed to open " BMA150_NAME);
        _exit(4);
    }
    mode = BMA_MODE_SLEEP;
    if (ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &mode) != 0) {
        perror("Failed to put bma150 to sleep initially.");
        _exit(5);
    }
}

int main(int argc, char **argv)
{
    state_t state = SLEEP;
    int i;
   
    if (argc != 8) {
        fprintf(stderr, "Usage: akmd <hx> <hy> <hz> <gx> <gy> <gz> <tz>\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "flux(i) = a * raw(i) * 10^(g(i)/8) + b * h(i), where\n");
        fprintf(stderr, "  h(i) = -128 .. 127\n");
        fprintf(stderr, "  g(i) = 0 .. 15\n");
        fprintf(stderr, "  a, b = internal scaling parameters\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "Attention ROM makers. The analog parameters hx, hy and hz can be left at 0.\n");
        fprintf(stderr, "Per-axis gain needs to be calibrated per device.\n");
        fprintf(stderr, "The Earth's magnetic field is approximately 45 uT and should\n");
        fprintf(stderr, "read the same in every orientation of device.\n");
        _exit(100);
    }

    /* args 1 .. 3, 4 .. 6 */
    for (i = 0; i < 3; i ++) {
        /* AKM specification says that values 0 .. 127 are monotonously
         * decreasing corrections, and values 128 .. 255 are
         * monotonously increasing corrections and that 0 and 128
         * are near each other. */
        int corr = atoi(argv[1+i]);
        if (corr < 0) {
            corr = 127 - corr;
        }
        /* now straightened so that -128 .. 127 can be used, center at 0 */
        analog_offset[i] = corr;
        analog_gain[i] = atoi(argv[4+i]);
    }
    temperature_zero = atoi(argv[7]);
 
    open_fds();
    calibrate_analog_apply();
    while (1) {
        switch (state) {
        case READ:
            state = readLoop();
            break;
        case SLEEP:
            state = sleepLoop();
            break;
        }
    }
}
