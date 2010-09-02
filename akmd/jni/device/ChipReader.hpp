#pragma once

#include "math/Vector.hpp"

namespace akmd {

class ChipReader {
    public:
    ChipReader() {}
    virtual ~ChipReader() {}

    /* Start continuous measurements. */
    virtual void start() = 0;
    /* Stop continuous measurements. */
    virtual void stop() = 0;

    /* Perform physical measurement and return 3-value triplet from chip.
     * If measured value is a vector, it must be in the Device Coordinate System.
     *
     * The values are normalized so that 720 means 1G for acceleration,
     * and 16 = 1 uT for magnetic field. (Libsensors defined limits.)
     * With temperature, x coordinate holds celcius value.
     */
    virtual Vector read() = 0;
    /* Get delay requested by java side, or -1 if this
     * reader doesn't know what the delay should be. */
    virtual int get_delay() = 0;
};

}
