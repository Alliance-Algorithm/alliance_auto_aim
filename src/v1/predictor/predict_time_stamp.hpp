#pragma once

#include "interfaces/time_stamped.hpp"

namespace world_exe::v1::predictor {
class PredictTimeStamp : public interfaces::ITimeStamped {
public:
    PredictTimeStamp() = default;
    PredictTimeStamp(const std::time_t& time_stamp)
        : time_stamp_(time_stamp) { }

    inline void SetTimeStamp(const time_t& time_stamp) { time_stamp_ = time_stamp; }

    const std::time_t& GetTimeStamp() const override { return time_stamp_; };

private:
    std::time_t time_stamp_;
};
}