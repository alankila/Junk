#pragma once

#include "device/AKM8973_2_6_29.hpp"
#include "device/ChipWriter.hpp"

namespace akmd {

class AKM8973_writer_2_6_29 : public ChipWriter {
    private:
    AKM8973_2_6_29* backend;

    public:
    AKM8973_writer_2_6_29(AKM8973_2_6_29* reader);
    ~AKM8973_writer_2_6_29();

    void write(short *data);
};

}
