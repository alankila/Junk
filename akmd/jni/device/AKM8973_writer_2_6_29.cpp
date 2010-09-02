#include "device/AKM8973_writer_2_6_29.hpp"

namespace akmd {

AKM8973_writer_2_6_29::AKM8973_writer_2_6_29(AKM8973_2_6_29* reader)
{
     this->backend = reader;
}

AKM8973_writer_2_6_29::~AKM8973_writer_2_6_29()
{
}

void AKM8973_writer_2_6_29::write(short *data) {
    backend->publish(data);
}

}
