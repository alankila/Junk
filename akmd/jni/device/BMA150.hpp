#pragma once

#include <sys/time.h>

#include "device/ChipReader.hpp"
#include "Calibrator.hpp"
#include "math/Vector.hpp"

#define BMA150_NAME "/dev/bma150"

namespace akmd {

class BMA150 : public ChipReader {
    private:
    /* Open file descriptors */
    int fd;

    Calibrator accelerometer;
    Vector accelerometer_g;

    struct timeval next_update;
    int index;
    Vector abuf[2];
    Vector a;

    private:
    void calibrate();

    public:
    BMA150();
    ~BMA150();

    void measure();
    Vector read();
    int get_delay();

    void start();
    void stop();
};

}
