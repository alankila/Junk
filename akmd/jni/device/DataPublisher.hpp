#pragma once

namespace akmd {

class DataPublisher {
    public:
    virtual ~DataPublisher() {}

    virtual void publish(short* data) = 0;
};

}
