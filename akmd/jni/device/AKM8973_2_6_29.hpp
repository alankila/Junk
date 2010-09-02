#pragma once

#include "device/ChipReader.hpp"
#include "device/DataPublisher.hpp"
#include "device/TemperatureReader.hpp"
#include "util.hpp"
#include "math/Vector.hpp"

namespace akmd {

class AKM8973_2_6_29 : public ChipReader, public TemperatureReader, public DataPublisher
{
    private:

    /* Temperature is -(value-zero). */
    char temperature_zero;
    /* The analog offset */
    signed char analog_offset[3];
    /* The user requested magnetometer gain */
    int magnetometer_gain;
    /* The actual gain used on hardware */
    int fixed_magnetometer_gain;
    /* Digital gain to compensate for analog setting. */
    float digital_gain;

    /* Misc. measurement data */
    int fd;
    short temperature;
    int index;
    Vector mbuf[2];

    float rc_min[3];
    float rc_max[3];

    char akm_analog_offset(int i);
    char calibrate_analog_apply();
    void calibrate_magnetometer_analog_helper(float val, int i);
    void calibrate_magnetometer_analog(Vector* m);

    public:
    AKM8973_2_6_29(int gain);
    ~AKM8973_2_6_29();

    int get_delay();
    Vector read();

    void start();
    void stop();
   
    /* AKM8973-specific addendums. */
    void publish(short *data);
    int get_temperature();

    void wait_start();
    void wait_stop();
};

}
