#pragma once

#include "device/ChipReader.hpp"
#include "device/TemperatureReader.hpp"

namespace akmd {

/*
 * The purpose of this class is to wrap a chip that
 * can also provide temperature sensing information as
 * a byproduct of another measurement. Only method that
 * should be implemented is read().
 */
class TemperatureReaderAdapter : public ChipReader {
    private:
    float zero;
    TemperatureReader* backend;

    public:
    TemperatureReaderAdapter(TemperatureReader* reader, float temperature_zero);
    ~TemperatureReaderAdapter();

    void measure() {}
    Vector read();
    int get_delay() { return -1; }

    void start() {}
    void stop() {}
};

}
