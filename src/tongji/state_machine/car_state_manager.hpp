#pragma once

#include <algorithm>

#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::car_state {

using TimeStamp = predictor::TimeStamp;
class CarStateManager {
public:
    CarStateManager(int switch_threshold = 5)
        : switch_threshold_(switch_threshold)
        , last_seen_(std::time(nullptr)) { }

    void Update(bool detected, const TimeStamp& now) {
        if (detected) {
            count_     = std::min(count_ + 1, switch_threshold_);
            last_seen_ = now;
            update_count_++;
            if (update_count_ >= converge_threshold_ && is_diverged_) is_converged_ = true;
        } else {
            count_ = std::max(count_ - 1, 0);
        }
        is_locked_ = (count_ >= switch_threshold_);
    }

    bool IsConverged() const { return is_converged_ && !is_diverged_; }

    bool IsLost(const TimeStamp& now) const {
        return is_locked_ && now.SecondsSince(last_seen_) > timeout_sec_;
    }

    void Reset() {
        count_        = 0;
        update_count_ = 0;
        is_locked_    = false;
        is_converged_ = false;
        is_diverged_  = true;
        priority_     = default_priority_;
        last_seen_    = predictor::TimeStamp(0);
    }

    void SetPriority(const int& p) { priority_ = p; }
    int GetPriority() const { return priority_; }

    void SetThreshold(const int& value) { switch_threshold_ = value; }

    predictor::TimeStamp LastSeen() const { return last_seen_; }

private:
    int count_ = 0;
    int switch_threshold_;
    int converge_threshold_ = 3;
    int update_count_       = 0;
    double timeout_sec_;

    bool is_locked_       = false;
    bool is_converged_    = false;
    bool is_diverged_     = true;
    int priority_         = 100;
    int default_priority_ = 100;

    predictor::TimeStamp last_seen_;
};
}