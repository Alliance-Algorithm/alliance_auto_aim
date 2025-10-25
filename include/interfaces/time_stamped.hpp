#pragma once

#include <ctime>
namespace world_exe::interfaces {
/**
 * @brief 时间戳，通常是SteadyClock
 */
class ITimeStamped {
public:
    virtual const std::time_t GetTimeStamp() const = 0;

    virtual ~ITimeStamped() = default;
};
}