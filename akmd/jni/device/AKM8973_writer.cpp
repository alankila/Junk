#include "device/AKM8973_writer.hpp"

namespace akmd {

AKM8973_writer::AKM8973_writer(DataPublisher* reader)
{
     this->backend = reader;
}

AKM8973_writer::~AKM8973_writer()
{
}

void AKM8973_writer::write(short *data) {
    backend->publish(data);
}

}
