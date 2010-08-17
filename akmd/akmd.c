/*
 * Free, open-source replacement of the closed source akmd driver written
 * by AKM. This program is the user-space counterpart of akm8973 and bma150
 * sensors found on various Android phones.
 *
 * Copyright Antti S. Lankila, 2010, licensed under the Apache License.
 *
 * The driver does not use akm8973's acceleration data. Instead, it
 * queries the bma150 sensor and reports its data instead.
 *
 * The ioctl to read data from is called akm8973_daemon. The control
 * device node is akm8973_aot. The libsensors from android talks to
 * akm8973_aot. The akm8973_daemon samples the chip data and performs
 * the analysis. The measurement is inherently a slow process, and therefore
 * cached copy of results is periodically updated to the /dev/input node
 * called "compass" using an ioctl on the akm8973_daemon.
 *
 * Here are some definitions of various orientations:
 *
 * If device is lying on its back on table:
 * x = the short edge of device, increasing towards right
 * y = the long endge of device, increasing towards top
 * z = the direction of sky, increasing towards sky
 *
 * Yaw (Azimuth): 0 points towards North, 90 to East, 180 to South, 270 to West
 * Pitch: rotation around X axis, with positive values when z axis moves
 * towards y axis ("down")
 * Roll: rotation around Y axis, with positive values when z axis moves
 * towards x axis ("right")
 *
 * libsensors flips the sign of Roll, acceleration A, acceleration Z,
 * magnetic field X, magnetic field Y.
 *
 * BMA150 reports acceleration like this:
 *
 * gravity vector z is negative when device lays flat on table.
 * when raised up into portrait orientation, y is positive.
 * when laying on the left edge, x is negative.
 *
 * Therefore BMA's y differs from the others, as y is incrementing
 * down the phone's edge rather than towards top.
 *
 * Magnetic sensor orientation has axis according to device coordinate system:
 * x decrements when approaching from direction of positive x (left)
 * y decrements when approaching from direction of positive y (keyboard)
 * z decrements when approaching from direction of positive z (below)
 * (I just tested with a magnetized screwdriver; it is possible that
 *  all axes are consistently flipped.)
 */
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
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

#define CALIBRATE_DECAY_SPEED 8
#define CALIBRATE_MAX_VALUE (128 << CALIBRATE_DECAY_SPEED)
#define CALIBRATE_STOP_DISTANCE (50 << CALIBRATE_DECAY_SPEED)
static int calibrate_min[3] = {  CALIBRATE_MAX_VALUE,  CALIBRATE_MAX_VALUE,  CALIBRATE_MAX_VALUE };
static int calibrate_max[3] = { -CALIBRATE_MAX_VALUE, -CALIBRATE_MAX_VALUE, -CALIBRATE_MAX_VALUE };

/* Establish the min-max bounds for every measurement seen so far with
 * decay function. */
static void calibrate(int *m)
{
    int i;

    for (i = 0; i < 3; i ++) {
        int val = m[i] << CALIBRATE_DECAY_SPEED;
        /* minimum value seen */
        if (calibrate_min[i] > val) {
            calibrate_min[i] = val;
        }
        /* maximum value seen */
        if (calibrate_max[i] < val) {
            calibrate_max[i] = val;
        }
        /* gradually move the minimum towards the positive infinity.
         * This allows us to recover from spikes like nearby magnetic
         * objects. */
        calibrate_min[i] += (val - calibrate_min[i]) / CALIBRATE_STOP_DISTANCE;
        calibrate_max[i] -= (calibrate_max[i] - val) / CALIBRATE_STOP_DISTANCE;

        /* Apply current estimate of the midpoint correction adjustment. */
        int correction = (calibrate_max[i] + calibrate_min[i]) >> (CALIBRATE_DECAY_SPEED + 1);
        m[i] -= correction;
    }

    /* We should also estimate the analog gain from examining the value
     * range in x, y and z directions, and assume that the magnetic field
     * being measured is the Earth's magnetic field and try to adjust
     * gain such that it gives 45. This could be seen to be cheating.
     * The alternative is to force ROM vendors to supply reasonable gain
     * values. */
}

static void open_fds(char *params)
{
    short mode;
    char rwbuf[5];
    int i;

    akm_fd = open(AKM_NAME, O_RDONLY);
    if (akm_fd == -1) {
        perror("Failed to open " AKM_NAME);
        _exit(1);
    }
    mode = AKECS_MODE_POWERDOWN;
    if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, &mode) != 0) {
        perror("Failed to put akm8973 to sleep");
        _exit(2);
    }

    for (i = 0; i < 6; i ++) {
        rwbuf[0] = 2;
        rwbuf[1] = 0xe1 + i;
        rwbuf[2] = params[i];
        if (ioctl(akm_fd, ECS_IOCTL_WRITE, &rwbuf) != 0) {
            perror("Failed to write gain controls\n");
            _exit(3);
        }
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

static state_t readLoop(char tz)
{
    unsigned int status;
    unsigned short delay;
    short mode;

    struct timespec interval;
    char akm_data[5];
    short bma150_data[7];
    short final_data[12];

    if (ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) != 0) {
        perror("akm8973: Failed to query control channel (aot) open count");
        _exit(10);
    }

    /* Nobody has aot socket open? We'll close the shop... */
    if (status == 0) {
        mode = AKECS_MODE_POWERDOWN;
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

    /* Measuring puts readable state to 0. It is going to take
     * some time before the values are ready. */
    mode = AKECS_MODE_MEASURE;
    if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, &mode) != 0) {
        perror("akm8973: Failed to SET_MODE=READ");
        _exit(13);
    }

    /* Significance and range of values can be extracted from bma150.c. */
    if (ioctl(bma150_fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data) != 0) {
        perror("bma150: Failed to READ_ACCELERATION");
        _exit(14);
    }
    int a[] = { bma150_data[0], -bma150_data[1], bma150_data[2] };

    /* Significance and range of values can be extracted from
     * online AKM 8973 manual. The kernel driver is dumb. */
    if (ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data) != 0) {
        perror("akm8973: Failed to GETDATA");
        _exit(15);
    }
    short temperature = (signed char) -(akm_data[1] + tz);
    int m[] = { 127 - (unsigned char) akm_data[2],
                127 - (unsigned char) akm_data[3],
                127 - (unsigned char) akm_data[4] };
  
    //LOGI("a=(%d %d %d), m=(%d %d %d)", a[0], a[1], a[2], m[0], m[1], m[2]);
    
    calibrate(m);
 
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
    float o2[3];
    float ref[3] = { 0, -1, 0 };
    /* a = 9 bits, o1 = 9 bits */
    cross_product(a, ref, o1);
    /* o2 = 18 bits */
    cross_product(a, o1, o2);
    
    /* Now project magnetic field on components o1 and o2. */
    /* m = 9 bits, o1 = 9 bits result 18 bits */
    float o1l = dot(m, o1) * length_f(o2);
    /* m = 9 bits, o2 = 18 bits result 27 bits */
    float o2l = dot(m, o2) * length_f(o1);

    /* Establish the angle in E */
    final_data[0] = 180.0f + atan2f(o1l, o2l) / (float) M_PI * 180.0f;
    /* pitch */
    final_data[1] = atan2f(a[1], -a[2]) / (float) M_PI * 180.0f;
    /* roll */
    final_data[2] = 90.0f - acosf(a[0] / length_i(a)) / (float) M_PI * 180.0f;
    
    final_data[3] = temperature;
    /* FIXME: how to establish accuracy? */
    final_data[4] = 3; // status of mag. sensor (UNRELIABLE, LOW, MEDIUM, HIGH)
    final_data[5] = 3; // status of acc. sensor (UNRELIABLE, LOW, MEDIUM, HIGH)

    // Android wants 720 = 1G, Device has 256 = 1G. */
    final_data[6] = (128 + a[0] * 720) >> 8;
    final_data[7] = (128 + a[2] * 720) >> 8;
    final_data[8] = (128 + a[1] * -720) >> 8;

    // CONVERT_M = 1/16 = 16 values = 1 uT.
    final_data[9]  =  m[0] << 4;
    final_data[10] =  m[1] << 4;
    final_data[11] = -m[2] << 4;
   
    /* Put data to be readable from compass input. */
    if (ioctl(akm_fd, ECS_IOCTL_SET_YPR, &final_data) != 0) {
        perror("bma150: Failed to SET_YPR={akm data}");
        _exit(16);
    }

    /* Get time until next update. */
    if (ioctl(akm_fd, ECS_IOCTL_GET_DELAY, &delay) != 0) {
         perror("akm8973: Failed to GET_DELAY");
        _exit(17);
    }

    /* We could actually use gettimeofday when we start to
     * achieve truly periodic timer tick. Right now we really
     * sleep for interval + processing time. */
    //delay = 500;
    interval.tv_sec = delay / 1000;
    interval.tv_nsec = 1000000 * (delay % 1000);
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

int main(int argc, char **argv)
{
    state_t state = SLEEP;
    char params[6];
    char tz;
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
         * monotonously increasing positive corrections and that 0 and 128
         * are near each other. */
        int corr = atoi(argv[1+i]);
        if (corr < 0) {
            corr = 127 - corr;
        }
        /* now straightened so that -128 .. 127 can be used, center at 0 */
        params[i] = corr;
        params[3+i] = atoi(argv[4+i]);
    }
    tz = atoi(argv[7]);
 
    open_fds(params);
    while (1) {
        switch (state) {
        case READ:
            state = readLoop(tz);
            break;
        case SLEEP:
            state = sleepLoop();
            break;
        }
    }
}
