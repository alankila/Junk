#include "device/AKM8973_temperature.hpp"

namespace akmd {

AKM8973_temperature::AKM8973_temperature(TemperatureReader* reader, float temperature_zero)
{
    this->backend = reader;
    this->zero = temperature_zero;
}

AKM8973_temperature::~AKM8973_temperature()
{
}

Vector AKM8973_temperature::read() {
    return Vector(-(backend->get_temperature() + zero), 0, 0);
}

int AKM8973_temperature::get_delay() {
    return -1;
}

void AKM8973_temperature::start() {
}

void AKM8973_temperature::stop() {
}

}
