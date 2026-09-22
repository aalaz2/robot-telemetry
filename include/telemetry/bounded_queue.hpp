#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace telemetry {
    template <typename T>
    class BoundedQueue {
        public:
        explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}
        bool TryPush(T item) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutting_down_) {
                return false;
            }
            if (queue_.size() >= capacity_) {
                ++dropped_count_;
                return false;
            }
            queue_.push_back(std::move(item));
            cv_.notify_one();
            return true;
        }
        std::optional<T> Pop () {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !queue_.empty() || shutting_down_; });
            if (queue_.empty()) {
                return std::nullopt;
            }
            T item = std::move(queue_.front());
            queue_.pop_front();
            return item;
        }
        void Shutdown() {
            std::lock_guard<std::mutex> lock(mutex_);
            shutting_down_ = true;
            cv_.notify_all();
        }

        std::size_t DroppedCount() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return dropped_count_;
        }

        std::size_t SizeApprox() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return queue_.size();
        }
        private:
        mutable std::mutex mutex_;
        std::condition_variable cv_;
        std::deque<T> queue_;
        std::size_t capacity_;
        std::size_t dropped_count_ = 0;
        bool shutting_down_ = false;
    };
}