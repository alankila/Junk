/*
 * Theory of operation:
 *
 * Init:
 * 1. open akm8973_daemon, bma150
 *
 * Mainloop:
 * 2. 8973: call GET_OPEN_STATUS to work out if we need to do anything
 *    if not, put akm8973 to powerdown state, goto 11
 * 3. 8973: call SET_MODE with READ parameter
 * 4. 8973: GETDATA, get datastruct
 * 5. bma150: READ_ACCELERATION
 * 6. Fill in bma150's acceleration data into 8973's datastruct
 * 7. 8973: SET_YPR with datastruct
 * 8. 8973: GET_DELAY to get delay value
 * 9. sleep(delay value)
 * 10. goto 2
 *
 * Sleep:
 * 11. sleep 1s.
 * 12. GET_OPEN_STATUS. If still closed, goto 11
 * 13. goto 3.
 *
 * Classic akmd is probably processing the returned data in some way
 * I'm just going to hand back the raw data in the first attempt to make
 * free akmd.
 */

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "akm8973.h"
#include "bma150.h"

#define AKM_NAME "/dev/akm8973_daemon"
#define BMA150_NAME "/dev/bma150"

static int akm_fd, bma150_fd;

typedef enum { READ, SLEEP } state_t;

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

static void cross_product(float ax, float ay, float az, float bx, float by, float bz, float *v)
{
    v[0] = ay * bz - az * by;
    v[1] = az * bx - ax * bz;
    v[2] = ax * by - ay * bx;
}

static void normalize(float *v)
{
    float norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + 1);
    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
}

static float dot(float ax, float ay, float az, float bx, float by, float bz)
{
    return ax * bx + ay * by + az * bz;
}

static state_t readLoop(char tz, signed char* digital_adjustment)
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

    /* Significance and range of values can be extracted from
     * bma150.c. */
    if (ioctl(bma150_fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data) != 0) {
        perror("bma150: Failed to READ_ACCELERATION");
        _exit(14);
    }
    short ax = bma150_data[0];
    short ay = bma150_data[1];
    short az = bma150_data[2];

    /* Absolutely no documentation is available about the result structure.
     * this could be just hardware register dump. */
    if (ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data) != 0) {
        perror("akm8973: Failed to GETDATA");
        _exit(15);
    }
    short temperature = (signed char) -(akm_data[1] + tz);
    short mx = 127 - (unsigned char) akm_data[2];
    short my = 127 - (unsigned char) akm_data[3];
    short mz = 127 - (unsigned char) akm_data[4];
    
    mx += digital_adjustment[0];
    my += digital_adjustment[1];
    mz += digital_adjustment[2];
    mz = -mz;

#if 1
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
    cross_product(-ax, ay, az, 0, -1, 0, o1);
    cross_product(-ax, ay, az, o1[0], o1[1], o1[2], o2);
    normalize(o1);
    normalize(o2);

    /* Now project magnetic field on components o1 and o2. */
    int o1l = dot(mx, my, mz, o1[0], o1[1], o1[2]);
    int o2l = dot(mx, my, mz, o2[0], o2[1], o2[2]);

    /* Establish the angle in E */
    final_data[0]  = 180.0f + atan2f(o1l, o2l) / (float) M_PI * 180.0f;
#else
    final_data[0] = 180.0f + atan2(-mx, my) / (float) M_PI * 180.0f;
#endif

    /* pitch */
    final_data[1] = -atan2f(ay, -az) / (float) M_PI * 180.0f;
    /* roll */
    final_data[2] = -acosf(ax / sqrtf(ax * ax + ay * ay + az * az + 1)) / M_PI * 180.0f + 90.0f;
    
    final_data[3]  = temperature;
    final_data[4]  = 3; // status of mag. sensor (UNRELIABLE, LOW, MEDIUM, HIGH)
    final_data[5]  = 3; // status of acc. sensor (UNRELIABLE, LOW, MEDIUM, HIGH)

    // Android wants 720 = 1G, Device has 256 = 1G
    final_data[6]  = (128 + ax * 720) >> 8;
    final_data[7]  = (128 + ay * 720) >> 8;
    final_data[8]  = (128 + az * 720) >> 8;

    // CONVERT_M = 1/16 = 16 values = 1 uT.
    final_data[9]  = mx << 4;
    final_data[10] = my << 4;
    final_data[11] = mz << 4;
   
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
    signed char digital_adjustment[3];
    char tz;
    int i;
   
    if (argc != 11) {
        fprintf(stderr, "Usage: akmd <hx> <hy> <hz> <gx> <gy> <gz> <dx> <dy> <dz> <tz>\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "flux(i) = a * raw(i) * 10^(g(i)/8) + b * h(i) + d(i), where\n");
        fprintf(stderr, "  h(i) = -128 .. 127\n");
        fprintf(stderr, "  g(i) = 0 .. 15\n");
        fprintf(stderr, "  d(i) = -128 .. 127\n");
        fprintf(stderr, "  a, b = internal scaling parameters\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "Calibration requires establishing measurements in a known field strength\n");
        fprintf(stderr, "and ensuring that as device is rotated, the average readout is 0,\n");
        fprintf(stderr, "and maximum value along each axis equals the known strength.\n");
        fprintf(stderr, "The Earth's magnetic field is about 50 uT.\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "tz is the temperature zero reference value.");
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
    /* params 7 .. 9 */
    for (i = 0; i < 3; i ++) {
        digital_adjustment[i] = atoi(argv[7+i]);
    }
    tz = atoi(argv[10]);
 
    open_fds(params);
    while (1) {
        switch (state) {
        case READ:
            state = readLoop(tz, digital_adjustment);
            break;
        case SLEEP:
            state = sleepLoop();
            break;
        }
    }
}
