#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <utility>

#include <Eigen/StdVector>

namespace world_exe::util::thread {
template <typename T, bool PopWhenFull = false> class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;
    explicit ThreadSafeQueue(
        size_t max_size, std::function<void(void)> full_handler = [] { })
        : max_size_(max_size)
        , full_handler_(std::move(full_handler)) { }

    void push(T&& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        // max_size_ == 0 means unbounded.
        if (max_size_ != 0 && queue_.size() >= max_size_) {
            if (PopWhenFull) {
                queue_.pop();
            } else {
                full_handler_();
                return;
            }
        }

        queue_.push(std::forward<T>(value));
        not_empty_condition_.notify_all();
    }

    void pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        if (queue_.empty()) {
            // std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
            return;
        }

        value = queue_.front();
        queue_.pop();
    }

    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty() || stopped_; });

        if (queue_.empty() && stopped_) {
            return std::nullopt;
        }

        T value = std::move(queue_.front());
        queue_.pop();
        return std::move(value);
    }

    T front() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        return queue_.front();
    }

    void back(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (queue_.empty()) {
            // std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
            return;
        }

        value = queue_.back();
    }

    bool empty() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    void clear() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
        not_empty_condition_.notify_all(); // 如果其他线程正在等待队列不为空，这样可以唤醒它们
    }

    void stop() {
        std::unique_lock<std::mutex> lock(mutex_);
        stopped_ = true;
        not_empty_condition_.notify_all(); // 唤醒所有等待者
    }

private:
    // Use aligned storage so Eigen objects inside T stay 16-byte aligned.
    std::queue<T, std::deque<T, Eigen::aligned_allocator<T>>> queue_;
    size_t max_size_ = 0;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_condition_;
    bool stopped_ = false;
    std::function<void(void)> full_handler_;
};

} // namespace tools

#endif // TOOLS__THREAD_SAFE_QUEUE_HPP
