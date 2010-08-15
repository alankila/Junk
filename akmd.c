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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/akm8973.h>
#include <linux/bma150.h>

#define AKM_NAME "/dev/akm8973_daemon"
#define BMA150_NAME "/dev/bma150"

static int akm_fd, bma150_fd;

static enum { READ, SLEEP } state;

static void open_fds()
{
    akm_fd = open(AKM_NAME, O_RDONLY);
    if (akm_fd == -1) {
        perror("Failed to open " AKM_NAME);
        _exit(1);
    }
    if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, AKECS_MODE_POWERDOWN) != 0) {
        perror("Failed to put akm8973 to sleep initially.");
        _exit(2);
    }

    bma150_fd = open(BMA150_NAME, O_RDONLY);
    if (bma150_fd == -1) {
        perror("Failed to open " BMA150_NAME);
        _exit(3);
    }
    if (ioctl(bma150_fd, BMA_IOCTL_SET_MODE, BMA_MODE_SLEEP) != 0) {
        perror("Failed to put bma150 to sleep initially.");
        _exit(4);
    }
}

static void readLoop()
{
    unsigned int status;
    unsigned short delay;

    struct timespec interval;
    signed char akm_data[RBUFF_SIZE + 1];
    short bma150_data[7];
    short final_data[12];

    if (ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) != 0) {
        perror("akm8973: Failed to query control channel (aot) open count");
        _exit(10);
    }

    /* Nobody has aot socket open? We'll close the shop... */
    if (status == 0) {
        if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, AKECS_MODE_POWERDOWN) != 0) {
            perror("akm8973: Failed to SET_MODE=POWERDOWN");
            _exit(11);
        }
        if (ioctl(bma150_fd, BMA_IOCTL_SET_MODE, BMA_MODE_SLEEP) != 0) {
            perror("bma150: Failed to SET_MODE=SLEEP");
            _exit(12);
        }

        state = SLEEP;
        return;
    }

    /* Measuring puts readable state to 0. It is going to take
     * some time before the values are ready. */
    if (ioctl(akm_fd, ECS_IOCTL_SET_MODE, AKECS_MODE_MEASURE) != 0) {
        perror("akm8973: Failed to SET_MODE=READ");
        _exit(13);
    }

    /* Significance and range of values can be extracted from
     * bma150.c. */
    if (ioctl(bma150_fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data) != 0) {
        perror("bma150: Failed to READ_ACCELERATION");
        _exit(14);
    }

    /* Absolutely no documentation is available about the result structure.
     * this could be just hardware register dump. */
    if (ioctl(akm_fd, ECS_IOCTL_GETDATA, &akm_data) != 0) {
        perror("akm8973: Failed to GETDATA");
        _exit(15);
    }

    final_data[0]  = 0; // magnetic yaw 0 .. 360
    final_data[1]  = 0; // magnetic pitch -180 .. 180
    final_data[2]  = 0; // magnetic roll -90, 90
    final_data[3]  = akm_data[3]; // temperature  -30 .. 85
    final_data[4]  = 3;           // status of mag. sensor -32768 .. 3 (UNRELIABLE, LOW, MEDIUM, HIGH)
    final_data[5]  = 3;           // status of acc. sensor -32768 .. 3 (UNRELIABLE, LOW, MEDIUM, HIGH)
    final_data[6]  = bma150_data[0] << 2; // acceleration X -1872 .. 1872
    final_data[7]  = bma150_data[1] << 2; // acceleration Y -1872 .. 1872
    final_data[8]  = bma150_data[2] << 2; // acceleration Z -1872 .. 1872
    final_data[9]  = akm_data[0] << 3; // magnetic X -2048 .. 2032
    final_data[10] = akm_data[1] << 3; // magnetic Y -2048 .. 2032
    final_data[11] = akm_data[2] << 3; // magnetic Z -2048 .. 2032
    
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
}

void sleepLoop()
{
    int status;
    struct timespec interval;

    /* Aot has been opened? Resume if so. */
    if (ioctl(akm_fd, ECS_IOCTL_GET_OPEN_STATUS, &status) != 0) {
        perror("akm8973: Failed to query control channel (AOT) open count");
        _exit(20);
    }

    if (status != 0) {
        // akm device is woken by readLoop().
        if (ioctl(bma150_fd, BMA_IOCTL_SET_MODE, BMA_MODE_NORMAL) != 0) {
            perror("bma150: Failed to SET_MODE=NORMAL");
            _exit(21);
        }
        state = READ;
        return;
    }

    interval.tv_sec = 1;
    interval.tv_nsec = 0;
    nanosleep(&interval, NULL);
}

int main(int argc, char **argv)
{
    state = SLEEP;
    
    open_fds();
    while (1) {
        switch (state) {
        case READ:
            readLoop();
            break;
        case SLEEP:
            sleepLoop();
            break;
        }
    }
}
