#pragma once

namespace akmd {

class TemperatureReader {
    public:
    virtual ~TemperatureReader() {}

    virtual int get_temperature() = 0;
};

}
