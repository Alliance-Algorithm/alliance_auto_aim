#pragma once

#include <algorithm>

#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::car_state {

class CarStateManager {
public:
    CarStateManager(int switch_threshold = 5)
        : switch_threshold_(switch_threshold) { }

    void Update(bool detected, std::time_t now_raw) {
        if (detected) {
            count_     = std::min(count_ + 1, switch_threshold_);
            auto now   = predictor::TimeStamp::FromRaw(now_raw);
            last_seen_ = now;
        } else {
            count_ = std::max(count_ - 1, 0);
        }
        is_locked_ = (count_ >= switch_threshold_);
    }

    bool IsLost(const predictor::TimeStamp& now, double timeout_sec) const {
        return now.SecondsSince(last_seen_) > timeout_sec;
    }

    bool IsLocked() const { return is_locked_; }
    bool IsConverged() const { return is_converged_; }
    bool IsDiverged() const { return is_diverged_; }

    void SetPriority(int p);
    void SetThreshold(const int& value) { switch_threshold_ = value; }
    void SetDiverged(bool diverged) {
        is_diverged_ = diverged;
        if (diverged) is_converged_ = false;
    }
    void SetPriority(const int& p) { priority_ = p; }
    int GetPriority() const { return priority_; }

    predictor::TimeStamp LastSeen() const { return last_seen_; }

    void IncrementUpdateCount() {
        update_count_++;
        if (update_count_ > 3 && !is_diverged_) {
            is_converged_ = true;
        }
    }

private:
    int count_ = 0;
    int switch_threshold_;
    int update_count_ = 0;

    bool is_locked_    = false;
    bool is_converged_ = false;
    bool is_diverged_  = false;
    int priority_      = 100; // 默认最低优先级
    predictor::TimeStamp last_seen_;
};
}