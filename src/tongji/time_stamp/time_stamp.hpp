#pragma once

#include "interfaces/time_stamped.hpp"

namespace world_exe::tongji::time_stamp {
class TimeStamp : public interfaces::ITimeStamped {
public:
    TimeStamp(const std::time_t& time_stamp)
        : time_stamp_(time_stamp) { }

    void SetTimeStamp(const std::time_t& time_stamp) { time_stamp_ = time_stamp; }

    double SecondsSince(const TimeStamp& other) const {
        return std::difftime(time_stamp_, other.time_stamp_);
    }
    const std::time_t& GetTimeStamp() const override { return time_stamp_; };

private:
    std::time_t time_stamp_;
};
}