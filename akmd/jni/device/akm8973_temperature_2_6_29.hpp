#pragma once

#include "device/akm8973_2_6_29.hpp"

namespace akmd {

class AKM8973_temperature_2_6_29 : public ChipReader {
    private:
    float zero;
    AKM8973_2_6_29* backend;

    public:
    AKM8973_temperature_2_6_29(AKM8973_2_6_29* reader, float temperature_zero);
    ~AKM8973_temperature_2_6_29();

    Vector read();
    int get_delay();

    void start();
    void stop();
};

}
