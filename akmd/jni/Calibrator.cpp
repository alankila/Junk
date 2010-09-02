/*
 * Copyright Antti S. Lankila, 2010, Licensed under GPL.
 */

#include <math.h>
#include <stdlib.h>

#include "Calibrator.hpp"
#include "math/Matrix.hpp"

namespace akmd {    

Calibrator::Calibrator(int validity)
{
    this->validity = validity;
    reset();
}

void Calibrator::reset()
{
    fit_time = 0;
    center = Vector(0, 0, 0);
    scale = Vector(1, 1, 1);
    memset(point_cloud, 0, sizeof(point_cloud));
}

Calibrator::~Calibrator()
{
}

#if PCR != 32
#error "PCR must be 32 for this version of classify()"
#endif
int Calibrator::classify(float v)
{
    int i = 0;
    float ref = -0.5f;
    while (v > ref && i < 3) {
        ref += 0.5f;
        i ++;
    }
    return i;
}

void Calibrator::update(int time, Vector a, Vector v)
{
    float len = a.length();
    if (len == 0) {
        return;
    }

    /* 3rd vector is not independent because we are normalized. It can
     * point above or below the xy plane, though, so total of 2+2+1 bits. */
    int idx = classify(a.x/len) << 3 | classify(a.y/len) << 1 | (a.z > 0 ? 1 : 0);

    point_cloud[idx].time = time;
    point_cloud[idx].v = v;
}

bool Calibrator::try_fit(int time)
{
    int n = 0;
    for (int i = 0; i < PCR; i ++) {
        if (point_cloud[i].time >= time - validity) {
            n ++;
        }
    }

    /* Less than half of bins filled with recent data? */
    if (n < PCR/2) {
        return false;
    }

    fit_time = time;

    Matrix a = Matrix(n, 6);
    Matrix b = Matrix(n, 1);

    n = 0;
    for (int i = 0; i < PCR; i ++) {
        if (point_cloud[i].time < time - validity) {
            continue;
        }

        Vector v = point_cloud[i].v;

        b.set(n, 0, -v.x*v.x);
        a.set(n, 0, -2.0f*v.x);
        a.set(n, 1, v.y*v.y);
        a.set(n, 2, -2.0f*v.y);
        a.set(n, 3, v.z*v.z);
        a.set(n, 4, -2.0f*v.z);
        a.set(n, 5, 1.0f);

        n ++;
    }

    float *x = Matrix::leastSquares(&a, &b);
    center = Vector(x[0], x[2] / x[1], x[4] / x[3]);
    scale = Vector(1, sqrtf(x[1]), sqrtf(x[3]));
    delete[] x;

    return true;
}

}
