#pragma once
#include "interfaces/time_stamped.hpp"
#include <chrono>
namespace world_exe::interfaces::detail {
class TimeStamped : public world_exe::interfaces::ITimeStamped {
public:
    TimeStamped()          = default;
    virtual ~TimeStamped() = default;
    const std::time_t& GetTimeStamp() const override;

private:
    std::time_t time_stamp_;
};
}