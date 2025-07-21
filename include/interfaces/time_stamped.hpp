#pragma once

#include <ctime>

#define COMBINE_TIME_STAMPED virtual const ITimeStamped& GetTimeStamped() const = 0;

namespace world_exe::interfaces {
class ITimeStamped {
public:
    virtual const std::time_t& GetTimeStamp() const = 0;
};
}