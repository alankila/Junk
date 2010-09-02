#pragma once

namespace akmd {

class ChipWriter {
    public:
    ChipWriter() {}
    virtual ~ChipWriter() {}

    virtual void write(short *data) = 0;
};

}
