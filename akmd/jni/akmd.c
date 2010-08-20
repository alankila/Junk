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

static struct timeval current_time;
static struct timeval next_update;

static int akm_fd, bma150_fd;

static char *axis_labels[] = { "X", "Y", "Z" };

/* Temperature is value - zero. */
static char temperature_zero = 0;
/* The actual gain used on hardware */
static int fixed_analog_gain = 15;
/* Digital gain to compensate for analog setting. */
static int digital_gain[3];
/* The user requested analog gain */
static int analog_gain[3];
/* The user requested analog offset */
static int analog_offset[3];

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "akmd.free", __VA_ARGS__)
#define succeed(...)\
    do { if ((__VA_ARGS__) != 0) {                                         \
        LOGI(__FILE__ " ioctl on line %d: %s", __LINE__, strerror(errno)); \
        exit(1);                                                           \
    } } while (0)

/****************************************************************************/
/* Some vector utilities                                                    */
/****************************************************************************/
static void cross_product_iff(int *a, float *b, float *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

static int dot_ii(int *a, int *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static float dot_if(int *a, float *b)
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

/****************************************************************************/
/* Analog calibration                                                       */
/****************************************************************************/
static void calibrate_analog_apply()
{
    char params[6] = {
        analog_offset[0], analog_offset[1], analog_offset[2],
        fixed_analog_gain, fixed_analog_gain, fixed_analog_gain,
    };

    int i;
    for (i = 0; i < 6; i ++) {
        char rwbuf[5] = { 2, 0xe1+i, params[i], 0, 0 };
        succeed(ioctl(akm_fd, ECS_IOCTL_WRITE, &rwbuf));
    }

    for (i = 0; i < 3; i ++) {
        int g = analog_gain[i];
        digital_gain[i] = powf(10.0f, (g - fixed_analog_gain) * 0.4f / 20.0f) * 65536.0f;
    }
}

void calibrate_analog(int *m)
{
    int i;
    for (i = 0; i < 3; i ++) {
        /* Analog clipping handling */
        if (m[i] == 127 || m[i] == -128) {
            analog_offset[i] += m[i] == 127 ? -1 : 1;
            LOGI("Adjusted analog axis %s to %d", axis_labels[i], analog_offset[i]);
            calibrate_analog_apply();
        }
    }
}

/****************************************************************************/
/* Digital calibration                                                      */
/****************************************************************************/
static void calibrate_digital_rough(int *m, int *rc)
{
    int i;
    const int CALIBRATE_DECAY = 8;
    static int rc_min[3] = { 0, 0, 0 };
    static int rc_max[3] = { 0, 0, 0 };

    for (i = 0; i < 3; i ++) {
        m[i] = m[i] * digital_gain[i] >> 12;

        /* gradually shrink ther angles */
        rc_min[i] += 1;
        rc_max[i] -= 1;

        /* minimum value seen */
        int value = m[i] << CALIBRATE_DECAY;
        if (rc_min[i] > value) {
            rc_min[i] = value;
        }

        /* maximum value seen */
        if (rc_max[i] < value) {
            rc_max[i] = value;
        }

        rc[i] = (rc_min[i] + rc_max[i]) >> (1 + CALIBRATE_DECAY);
    }
}

#define PCR 32
static void calibrate_digital_fine_update(int pc[PCR][3], int *m, int *rc)
{
    /* Record current sample at point cloud using the rough estimate to
     * determine which bin to put the precise measurement in. */
    int rm[3] = {
        m[0] - rc[0],
        m[1] - rc[1],
        m[2] - rc[2],
    };

    /* keep top 2 bits of normalized rm, and turn it into index */
    int len = (int) length_i(rm) / 4 + 1;
    rm[0] /= len;
    rm[1] /= len;
    /* 3rd vector is not independent because we are normalized. It can
     * point above or below the xy plane, though, givin total of 2+2+1 bits. */
    int idx = ((rm[0] & 3) << 3) | ((rm[1] & 3) << 1) | (rm[2] < 0 ? 1 : 0);

    int *pos = pc[idx];
    pos[0] = m[0];
    pos[1] = m[1];
    pos[2] = m[2];
}

static float calibrate_digital_fine_fit_eval(int pc[][3], float x, float y, float z, float r)
{
    int i;

    float error = 0;
    for (i = 0; i < PCR; i ++) {
        int *v = pc[i];

        float dx = v[0] - x;
        float dy = v[1] - y;
        float dz = v[2] - z;

        float d = sqrtf(dx * dx + dy * dy + dz * dz) - r;
        error += d * d;
    }
    return sqrtf(error);
}

static void calibrate_digital_fine_fit(int pc[][3], float *fc)
{
    /* Region to use for derivate estimation, 16 = 1 uT */
    const float d = 1;

    float x = fc[0];
    float y = fc[1];
    float z = fc[2];
    float r = fc[3];

    float error = calibrate_digital_fine_fit_eval(pc, x - d/2, y - d/2, z - d/2, r - d/2);
    float dx = calibrate_digital_fine_fit_eval(pc, x+d, y, z, r) - error;
    float dy = calibrate_digital_fine_fit_eval(pc, x, y+d, z, r) - error;
    float dz = calibrate_digital_fine_fit_eval(pc, x, y, z+d, r) - error;
    float dr = calibrate_digital_fine_fit_eval(pc, x, y, z, r+d) - error;

    /* Steepest descent */
    fc[0] -= dx/d;
    fc[1] -= dy/d;
    fc[2] -= dz/d;
    fc[3] -= dr/d;
}

static void calibrate_digital(int *m)
{
    /* Get approximate calibration data for sphere fitting algorithm. */
    static int rough_calibration[3];
    calibrate_digital_rough(m, rough_calibration);

    /* Use sphere fitting algorithm to do finer calibration. */
    static int point_cloud[PCR][3];
    static float fine_calibration[4] = { 0, 0, 0, 0 };
    calibrate_digital_fine_update(point_cloud, m, rough_calibration);
    calibrate_digital_fine_fit(point_cloud, fine_calibration);

    /* Adjust magnetic. */
    m[0] -= fine_calibration[0];
    m[1] -= fine_calibration[1];
    m[2] -= fine_calibration[2];
}

/****************************************************************************/
/* Sensor output calculation                                                */
/****************************************************************************/
static void estimate_earth(int *a, int *m, int *g)
{
    int i;
    /* When device is not being accelerated, g = a is true. However, when
     * device is being accelerated, we should check acceleration against the
     * magnetometer to differentiate rotation of device (where field vector
     * is turning also) from changes in position (where magnetic field is
     * unchanged).
     */

    for (i = 0; i < 3; i ++) {
        g[i] = a[i];
    }
}

static void build_result_vector(int *a, short temperature, int *m, short *out)
{
    calibrate_analog(m);
    calibrate_digital(m);
 
    static int g[3];
    estimate_earth(a, m, g);

    /*
     * I define yaw in the tangent plane E of the Earth, where direction
     * o2 points towards magnetic North.
     * measured within the tangent space.
     */

    /* From g, we need to discover 2 suitable vectors. Cross product
     * is used to establish orthogonal basis in E. */
    float o1[3];
    float ref[3] = { 0, -1, 0 };
    cross_product_iff(g, ref, o1);
    float o2[3];
    cross_product_iff(g, o1, o2);
    
    /* Now project magnetic field on components o1 and o2. */
    float o1l = dot_if(m, o1) * length_f(o2);
    float o2l = dot_if(m, o2) * length_f(o1);

    /* Establish the angle in E */
    out[0] = 180.0f + atan2f(o1l, o2l) / (float) M_PI * 180.0f;
    /* pitch */
    out[1] = atan2f(g[1], -g[2]) / (float) M_PI * 180.0f;
    /* roll */
    out[2] = 90.0f - acosf(g[0] / length_i(g)) / (float) M_PI * 180.0f;
    
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

/****************************************************************************/
/* Raw mechanics of sensor reading                                          */
/****************************************************************************/
void sleep_until_next_update()
{
    /* Get time until next update. */
    unsigned short delay;
    succeed(ioctl(akm_fd, ECS_IOCTL_GET_DELAY, &delay));

/* Automatic sampling rate adapation. The code is disabled because
 * the BMA150 chip's measurement is really pretty noisy. Enabling this
 * is appropriate only if it gets better smoothed by us. */
#if 0
    /* Number of updates we want per second.
     * 6 = 1500, 5 = 750, 4 = 375, etc. Chosen sample rate is
     * always equal or the nearest lower rate to what app wanted. */
    int required_sample_rate = delay != 0 ? 1000 / delay : 1500;
    int bma_delay_value = 6;
    int bma_delay_rate = 1500;
    while (bma_delay_value > 0 && required_sample_rate <= bma_delay_value) {
        bma_delay_value /= 2;
        bma_delay_value --;
    }

    char rwbuf[8] = { 2, RANGE_BWIDTH_REG, 0 };
    succeed(ioctl(bma150_fd, BMA_IOCTL_READ, &rwbuf));
    rwbuf[2] = (rwbuf[1] & 0xf8) | bma_delay_value;
    rwbuf[1] = RANGE_BWIDTH_REG;
    succeed(ioctl(bma150_fd, BMA_IOCTL_WRITE, &rwbuf));
#endif

    /* Find out how long to sleep so that we achieve true periodic tick. */ 
    succeed(gettimeofday(&current_time, NULL));
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

    struct timespec interval = {
        .tv_sec = sleep_time / 1000,
        .tv_nsec = 1000000 * (sleep_time % 1000),
    };
    succeed(nanosleep(&interval, NULL));
}

static void readLoop()
{
    /* This ioctl sleeps if the control channel isn't open, so we can use
     * this to pause ourselves. Another thread shuts down the BMA150 chip.
     * AKM 8973 will sleep when not being measured anyway. */
    int status;
    succeed(ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status));

    /* Measuring puts readable state to 0. It is going to take
     * some time before the values are ready. */
    short amode = AKECS_MODE_MEASURE;
    succeed(ioctl(akm_fd, ECS_IOCTL_SET_MODE, &amode));

    /* BMA150 is constantly measuring and filtering, so it never sleeps.
     * The ioctl in truth returns only 3 values, but buffer in kernel is
     * defined as 8 shorts long. */
    int a[3];
    short bma150_data[8];
    succeed(ioctl(bma150_fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data));
    a[0] = bma150_data[0];
    a[1] = -bma150_data[1];
    a[2] = bma150_data[2];

    /* Significance and range of values can be extracted from
     * online AK 8973 manual. The kernel driver just passes the data on. */
    int m[3];
    char akm_data[5];
    succeed(ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data));
    short temperature = (signed char) -(akm_data[1] + temperature_zero);
    m[0] = 127 - (unsigned char) akm_data[2];
    m[1] = 127 - (unsigned char) akm_data[3];
    m[2] = 127 - (unsigned char) akm_data[4];

    /* Calculate and set data readable on compass input. */
    short final_data[12];
    build_result_vector(a, temperature, m, final_data);
    succeed(ioctl(akm_fd, ECS_IOCTL_SET_YPR, &final_data));

    sleep_until_next_update();
}

static void open_fds()
{
    akm_fd = open(AKM_NAME, O_RDONLY);
    if (akm_fd == -1) {
        perror("Failed to open " AKM_NAME);
        _exit(1);
    }
    succeed(ioctl(akm_fd, ECS_IOCTL_RESET, NULL));
    calibrate_analog_apply();

    bma150_fd = open(BMA150_NAME, O_RDONLY);
    if (bma150_fd == -1) {
        perror("Failed to open " BMA150_NAME);
        _exit(1);
    }
    succeed(ioctl(bma150_fd, BMA_IOCTL_INIT, NULL));
}

void *aot_tracking_thread(void *arg)
{
    while (1) {
        int status;
        char bmode;

        /* When open, we enable BMA and wait for close event. */
        succeed(ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status));
        bmode = BMA_MODE_NORMAL;
        succeed(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode));

        /* When closed, we disable BMA and wait for open event. */
        succeed(ioctl(akm_fd, ECS_IOCTL_GET_CLOSE_STATUS, &status));
        bmode = BMA_MODE_SLEEP;
        succeed(ioctl(bma150_fd, BMA_IOCTL_SET_MODE, &bmode));
    }
}

int main(int argc, char **argv)
{
    int i;
   
    if (argc != 8) {
        printf("Usage: akmd <hx> <hy> <hz> <gx> <gy> <gz> <tz>\n");
        printf("\n");
        printf("flux(i) = a * raw(i) * 10^(g(i)/8) + b * h(i), where\n");
        printf("  h(i) = -128 .. 127\n");
        printf("  g(i) = 0 .. 15\n");
        printf("  a, b = internal scaling parameters\n");
        printf("\n");
        printf("Attention ROM makers. The analog parameters hx, hy and hz can be left at 0.\n");
        printf("Per-axis gain needs to be calibrated per device.\n");
        printf("The Earth's magnetic field is approximately 45 uT and should\n");
        printf("read the same in every orientation of device.\n");
        _exit(1);
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
    succeed(gettimeofday(&next_update, NULL));

    pthread_t thread_id;
    succeed(pthread_create(&thread_id, NULL, aot_tracking_thread, NULL));
    while (1) {
        readLoop();
    }
}
