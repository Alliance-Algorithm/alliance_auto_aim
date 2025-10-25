#pragma once

#include <bit>
#include <chrono>
#include <cstdint>

#include "interfaces/time_stamped.hpp"

namespace world_exe::tongji::time_stamp {
using namespace std::chrono;
class TimeStamp final : public interfaces::ITimeStamped {
public:
    using StorageType  = int64_t;
    using ClockType    = std::chrono::steady_clock;
    using TimeDuration = nanoseconds;

    explicit TimeStamp(const steady_clock::time_point& tp)
        : value_ns_(duration_cast<TimeDuration>(tp.time_since_epoch()).count()) { }
    explicit TimeStamp(const StorageType& ns_count)
        : value_ns_(ns_count) { }

    auto DeltaTime(const TimeStamp& other) const -> double {
        TimeDuration diff_ns = TimeDuration(value_ns_ - other.value_ns_);
        return std::chrono::duration<double>(diff_ns).count();
    }

    const std::time_t GetTimeStamp() const override {
        return std::bit_cast<std::time_t>(value_ns_);
    }

private:
    const StorageType value_ns_;
};
}