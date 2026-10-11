#ifndef EXPLORER_SIM_MAILBOX_HPP
#define EXPLORER_SIM_MAILBOX_HPP

#include <mutex>
#include <optional>
#include <utility>

namespace explorer_sim {

    // The latest-value mailbox from Part 2 of the book ("Condition Variables"), unchanged.
    //
    // One thread overwrites the value, another reads whatever is newest. Nobody waits
    // and nothing queues up: if the reader is slow it simply misses the values in
    // between. That is exactly right for data where only the newest sample matters:
    // a velocity command, a camera frame, a tuning value.
    //
    // For data where every item counts, use ThreadSafeQueue instead.
    template<typename T>
    class Mailbox {
    public:
        void set(T value) {// producer: overwrite with the newest sample
            std::lock_guard<std::mutex> lock(mutex_);
            value_ = std::move(value);
            hasValue_ = true;
        }

        std::optional<T> latest() const {// consumer: read whatever is newest (or nullopt)
            std::lock_guard<std::mutex> lock(mutex_);
            if (!hasValue_) {
                return std::nullopt;
            }
            return value_;
        }

    private:
        mutable std::mutex mutex_;
        T value_{};
        bool hasValue_ = false;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_MAILBOX_HPP
