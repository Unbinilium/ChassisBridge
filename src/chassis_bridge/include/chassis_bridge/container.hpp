#pragma once

#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <deque>
#include <utility>

namespace cb::container {
    namespace ts {
        template <typename T>
        class deque {
        public:
            deque()  = default;
            ~deque() = default;

            void push_back(T&& data) {
                std::unique_lock lock(mutex_);
                data_.push_back(std::forward<T>(data));
                cv_.notify_one();
            }

            auto front() const {
                std::shared_lock lock(mutex_);
                return data_.front();
            }

            void pop_front() {
                std::unique_lock lock(mutex_);
                data_.pop_front();
            }

            auto size() const {
                std::shared_lock lock(mutex_);
                return data_.size();
            }

            auto empty() const {
                std::shared_lock lock(mutex_);
                return data_.empty();
            }

            void wait() {
                while (empty()) {
                    std::unique_lock lock(cv_mutex_);
                    cv_.wait(lock);
                }
            }

        private:
            mutable std::shared_mutex       mutex_;
            mutable std::mutex              cv_mutex_;
            mutable std::condition_variable cv_;
            std::deque<T>                   data_;
        };
    };
};
