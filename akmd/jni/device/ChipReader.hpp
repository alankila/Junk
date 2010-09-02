#pragma once

#include "vector.hpp"

namespace akmd {

class ChipReader {
    public:
    ChipReader() {}
    virtual ~ChipReader() {}

    virtual void start() = 0;
    virtual void stop() = 0;

    virtual Vector read() = 0;
    virtual int get_delay() = 0;
};

}
