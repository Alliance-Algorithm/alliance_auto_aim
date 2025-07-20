#pragma once

#include <ctime>

#define COMBINE_TIME_STAMPED virtual const ITimeStamped& GetTimeStamped() const;

namespace world_exe::interfaces {
class ITimeStamped {
public:
    virtual const std::time_t& GetTimeStamp() const;
};
}