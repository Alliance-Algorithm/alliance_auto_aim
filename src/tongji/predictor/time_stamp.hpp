#pragma once

#include "interfaces/time_stamped.hpp"

namespace world_exe::tongji::predictor {
class TimeStamp : public interfaces::ITimeStamped {
public:
    TimeStamp() = default;
    TimeStamp(const std::time_t& time_stamp)
        : time_stamp_(time_stamp) { }

    static TimeStamp FromRaw(std::time_t raw) { return TimeStamp(raw); }

    inline void SetTimeStamp(const time_t& time_stamp) { time_stamp_ = time_stamp; }

    double SecondsSince(const TimeStamp& other) const {
        return std::difftime(time_stamp_, other.time_stamp_);
    }
    const std::time_t& GetTimeStamp() const override { return time_stamp_; };

private:
    std::time_t time_stamp_;
};
}