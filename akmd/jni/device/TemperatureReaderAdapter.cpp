#include "device/TemperatureReaderAdapter.hpp"

namespace akmd {

TemperatureReaderAdapter::TemperatureReaderAdapter(TemperatureReader* reader, float temperature_zero)
{
    this->backend = reader;
    this->zero = temperature_zero;
}

TemperatureReaderAdapter::~TemperatureReaderAdapter()
{
}

Vector TemperatureReaderAdapter::read() {
    return Vector(-(backend->get_temperature() + zero), 0, 0);
}

}
