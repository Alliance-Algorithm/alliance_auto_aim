#pragma once

#include <ctime>

#define COMBINE_TIME_STAMPED virtual const ITimeStamped& GetTimeStamped() const = 0

namespace world_exe::interfaces {
/**
 * @brief 时间戳，通常是SteadyClock
 */
class ITimeStamped {
public:
    virtual const std::time_t& GetTimeStamp() const = 0;

    virtual ~ITimeStamped() = default;
};
}