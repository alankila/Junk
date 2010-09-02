#include <fcntl.h>

#include "device/BMA150.hpp"
#include "linux-2.6.29/bma150.h"
#include "util.hpp"

namespace akmd {

BMA150::BMA150()
: index(0)
{
    abuf[0] = abuf[1] = Vector();

    fd = open(BMA150_NAME, O_RDONLY);
    SUCCEED(fd != -1);
    SUCCEED(ioctl(fd, BMA_IOCTL_INIT, NULL) == 0);
    
    char rwbuf[8] = { 1, RANGE_BWIDTH_REG };
    SUCCEED(ioctl(fd, BMA_IOCTL_READ, &rwbuf) == 0);
    rwbuf[2] = (rwbuf[1] & 0xf8) | 1; /* 47 Hz sampling */
    rwbuf[0] = 2;
    rwbuf[1] = RANGE_BWIDTH_REG;
    SUCCEED(ioctl(fd, BMA_IOCTL_WRITE, &rwbuf) == 0);
}

BMA150::~BMA150() {
    SUCCEED(close(fd) == 0);
}

int BMA150::get_delay() {
    return -1;
}

Vector BMA150::read()
{
    /* BMA150 is constantly measuring and filtering, so it never sleeps.
     * The ioctl in truth returns only 3 values, but buffer in kernel is
     * defined as 8 shorts long. */
    short bma150_data[8];
    SUCCEED(ioctl(fd, BMA_IOCTL_READ_ACCELERATION, &bma150_data) == 0);
    abuf[index] = Vector(bma150_data[0], -bma150_data[1], bma150_data[2]);
    index = (index + 1) & 1;

    return abuf[0].add(abuf[1]).multiply(0.5f);
}

void BMA150::start()
{
    char bmode = BMA_MODE_NORMAL;
    SUCCEED(ioctl(fd, BMA_IOCTL_SET_MODE, &bmode) == 0);
}

void BMA150::stop()
{        
    char bmode = BMA_MODE_SLEEP;
    SUCCEED(ioctl(fd, BMA_IOCTL_SET_MODE, &bmode) == 0);
}

}
