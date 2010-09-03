/* Vector arithmetics support.
 * Copyright Antti S. Lankila, 2010, licensed under the GPL.
 */

#include <math.h>

#include "math/Vector.hpp"

namespace akmd {

Vector::Vector()
{
    x = 0;
    y = 0;
    z = 0;
}

Vector::Vector(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Vector Vector::multiply(float k)
{
    return Vector(x * k, y * k, z * k);
}

Vector Vector::multiply(Vector o)
{
    return Vector(x * o.x, y * o.y, z * o.z);
}

Vector Vector::divide(float k)
{
    return multiply(1.0f / k);
}

Vector Vector::add(Vector o) {
    return Vector(x + o.x, y + o.y, z + o.z);
}

Vector Vector::cross(Vector o)
{
    return Vector(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x
    );
}

float Vector::dot(Vector o)
{
    return x * o.x + y * o.y + z * o.z;
}

float Vector::length()
{
    return sqrtf(dot(*this));
}

}
