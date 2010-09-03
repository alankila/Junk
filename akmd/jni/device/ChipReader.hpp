#pragma once

#include "math/Vector.hpp"

namespace akmd {

/* Represents a sensor chip that can be read,
 * or a virtual chip if the values are determined
 * from another ChipReader (e.g. TemperatureReaderAdapter).
 */
class ChipReader {
    public:
    ChipReader() {}
    virtual ~ChipReader() {}

    /* Start continuous measurements. */
    virtual void start() = 0;

    /* Stop continuous measurements. */
    virtual void stop() = 0;

    /* Perform physical measurement, store a 3-value triplet from chip. */
    virtual void measure() = 0;

    /* Return measurement data. This method can be called many times per measurement. */
    virtual Vector read() = 0;

    /* Get delay requested by java side, or -1 if this
     * reader doesn't know what the delay should be. */
    virtual int get_delay() = 0;
};

}
