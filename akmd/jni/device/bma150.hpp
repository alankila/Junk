#pragma once

#include "vector.hpp"
#include "device/ChipReader.hpp"

#define BMA150_NAME "/dev/bma150"

namespace akmd {

class BMA150 : public ChipReader {
    private:
    /* Open file descriptors */
    int fd;

    int index;
    Vector abuf[2];

    public:
    BMA150();
    ~BMA150();

    Vector read();
    int get_delay();

    void start();
    void stop();
};

}
