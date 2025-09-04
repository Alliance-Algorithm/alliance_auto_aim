#pragma once
#include "sync/sync_data.hpp"
#include <atomic>
#include <functional>
#include <opencv2/core/mat.hpp>
#include <thread>
#include <utility>

template <class T, typename... Args>
    requires std::is_base_of_v<world_exe::core::SyncData<Args...>, T>
class DoubleBuffer {
public:
    DoubleBuffer(T* a, T* b, const std::function<void(std::tuple<Args&...>)>& ready_func)
        : front_buffer_ { std::move(a) }
        , back_buffer_ { std::move(b) }
        , ready_ { false }
        , writing_ { false }
        , ready_func_ { std::move(ready_func) } {
        thread_ = std::thread { [this]() {
            while (true) {
                trigger();
                ready_func_(load());
            }
        } };
    }

    ~DoubleBuffer() {
        bool expected = false;
        if (writing_.compare_exchange_strong(expected, true)) {
            delete front_buffer_;
            delete back_buffer_;
        }
        thread_.join();
    }

    std::tuple<Args&...> load() { return front_buffer_->load(); }

    void store(Args... args) {
        bool expected = false;
        while (!writing_.compare_exchange_strong(expected, true))
            expected = false;
        back_buffer_->store(args...);
        ready_.store(true);
        writing_.store(false);
    }

    void trigger() {
        bool expected = true;
        while (!ready_.compare_exchange_strong(expected, false))
            expected = true;
        expected = false;
        while (!writing_.compare_exchange_strong(expected, true))
            expected = false;
        std::swap(front_buffer_, back_buffer_);
        writing_.store(false);
    }

private:
    T* front_buffer_;
    T* back_buffer_;
    std::atomic_bool ready_;
    std::atomic_bool writing_;
    const std::function<void(std::tuple<Args&...>)> ready_func_;
    std::thread thread_;
};