#pragma once

#include <stdlib.h>
#include <sys/time.h>

#include "Calibrator.hpp"
#include "device/ChipReader.hpp"
#include "device/ChipWriter.hpp"
#include "util.hpp"
#include "math/Vector.hpp"

namespace akmd {

class Akmd {
    private:
    /* Magnetic */
    ChipReader* magnetometer_reader;
    Calibrator magnetometer;
  
    /* Accelerometer */  
    ChipReader* accelerometer_reader;
    Calibrator accelerometer;
    Vector accelerometer_g;
    
    /* Temperature */
    ChipReader* temperature_reader;

    /* Data publishing */
    ChipWriter* result_writer;

    /* Desired start point of next measurement interval. */
    struct timeval next_update;

    /* Direction to Earth center */
    Vector earth;
    
    void calibrate_magnetometer(Vector a, Vector* m);
    void calibrate_accelerometer(Vector* a);
    void estimate_earth(Vector a);
    void fill_result_vector(Vector a, Vector m, short temperature, short* out);

    public:
    Akmd(ChipReader* a, ChipReader* m, ChipReader* t, ChipWriter* r);
    ~Akmd();
    void measure();
    void sleep_until_next_update();

    void start();
    void stop();
};

}
