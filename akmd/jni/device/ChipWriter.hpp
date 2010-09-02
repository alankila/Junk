#pragma once

namespace akmd {

/* FIXME: declare a struct for handling data cleanly in core */

class ChipWriter {
    public:
    ChipWriter() {}
    virtual ~ChipWriter() {}

    /* Write measurement data to wherever * libsensors can pick it up. The
     * array is sequence of values such that libsensors can read it. */
    virtual void write(short *data) = 0;
};

}
