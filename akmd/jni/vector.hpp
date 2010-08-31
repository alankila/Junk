#pragma once

namespace akmd {

class Vector {
    public:
    float x, y, z;

    Vector();
    Vector(float x, float y, float z);
    Vector add(Vector o);
    Vector multiply(float k);
    Vector multiply(Vector o);
    float dot(Vector o);
    Vector cross(Vector o);
    float length();
};

}
