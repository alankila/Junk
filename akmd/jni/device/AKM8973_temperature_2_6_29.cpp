#include "device/AKM8973_temperature_2_6_29.hpp"

namespace akmd {

AKM8973_temperature_2_6_29::AKM8973_temperature_2_6_29(AKM8973_2_6_29* reader, float temperature_zero)
{
    this->backend = reader;
    this->zero = temperature_zero;
}

AKM8973_temperature_2_6_29::~AKM8973_temperature_2_6_29()
{
}

Vector AKM8973_temperature_2_6_29::read() {
    return Vector(-(backend->get_temperature() + zero), 0, 0);
}

int AKM8973_temperature_2_6_29::get_delay() {
    return -1;
}

void AKM8973_temperature_2_6_29::start() {
}

void AKM8973_temperature_2_6_29::stop() {
}

}
