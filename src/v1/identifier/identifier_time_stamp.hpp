#pragma once

#include "interfaces/time_stamped.hpp"

namespace world_exe::identifier {
class IdentifierTimeStamp : public interfaces::ITimeStamped {
public:
    virtual const std::time_t& GetTimeStamp() const { return time_stamp; };

private:
    std::time_t time_stamp { 0 };
};
}