#pragma once

#include "device/ChipWriter.hpp"
#include "device/DataPublisher.hpp"

namespace akmd {

class AKM8973_writer : public ChipWriter {
    private:
    DataPublisher* backend;

    public:
    AKM8973_writer(DataPublisher* reader);
    ~AKM8973_writer();

    void write(short *data);
};

}
