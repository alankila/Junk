#pragma once

#include "device/ChipReader.hpp"

namespace akmd {

/*
 * The purpose of this class is to wrap a chip that
 * can calculate orientation information from other
 * sources. This one determines orientation from low-passed
 * acceleration signal.
 */
class OrientationAdapter : public ChipReader {
    private:
    ChipReader* accelerometer;
    ChipReader* magnetometer;
    Vector earth;

    public:
    OrientationAdapter(ChipReader* a, ChipReader *m);
    ~OrientationAdapter();
 
    void measure() {}
    Vector read();
    int get_delay() { return -1; }

    void start() {}
    void stop() {}
};

}
