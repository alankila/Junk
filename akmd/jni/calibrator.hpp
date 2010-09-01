#pragma once

#include "vector.hpp"

namespace akmd {

#define PCR 32 /* record this number of vectors around the device */

typedef struct {
    Vector v;
    int time;
} point_t;

class Calibrator {
    private:
    int validity;
    point_t point_cloud[PCR];

    static int classify(float value);

    public:
    Calibrator(int validity);
    ~Calibrator();

    void update(int time, Vector bin, Vector value);
    bool try_fit(int time, Vector *fc);
    void reset();
};

}
