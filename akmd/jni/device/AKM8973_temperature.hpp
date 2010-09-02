#pragma once

#include "device/ChipReader.hpp"
#include "device/TemperatureReader.hpp"

namespace akmd {

class AKM8973_temperature : public ChipReader {
    private:
    float zero;
    TemperatureReader* backend;

    public:
    AKM8973_temperature(TemperatureReader* reader, float temperature_zero);
    ~AKM8973_temperature();

    Vector read();
    int get_delay();

    void start();
    void stop();
};

}
