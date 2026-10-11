#ifndef EXPLORER_SIM_THREAD_SAFE_QUEUE_HPP
#define EXPLORER_SIM_THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <utility>

namespace explorer_sim {

    // The thread-safe queue from Part 2 of the book ("Condition Variables"), with one
    // addition: tryPop(), for a consumer that must never block.
    //
    // Use it where every item counts and order matters. Two places in this program:
    //   * telemetry out: the simulator pushes one sample per sensor tick, the publisher
    //     thread sleeps in waitAndPop() until one arrives.
    //   * jobs in: a ROS service callback pushes a job, the render loop picks jobs up
    //     with tryPop() once per frame (it has a frame to draw and cannot wait).
    template<typename T>
    class ThreadSafeQueue {
    public:
        void push(T value) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                queue_.push(std::move(value));
            }
            cv_.notify_one();// wake one waiting consumer
        }

        // Blocks until an item is available, or until the queue is closed and empty.
        // Returns std::nullopt only in that "closed and drained" case, so a consumer
        // loop can end cleanly instead of waiting forever for data that will never come.
        std::optional<T> waitAndPop() {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !queue_.empty() || closed_; });
            if (queue_.empty()) {// reachable only once closed_ is true
                return std::nullopt;
            }
            T value = std::move(queue_.front());
            queue_.pop();
            return value;
        }

        // Never blocks: an item if one is waiting, std::nullopt otherwise.
        std::optional<T> tryPop() {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.empty()) {
                return std::nullopt;
            }
            T value = std::move(queue_.front());
            queue_.pop();
            return value;
        }

        // Signal that no more items will arrive. Wakes every waiter so each drains
        // what is left and then sees nullopt and stops.
        void close() {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                closed_ = true;
            }
            cv_.notify_all();
        }

    private:
        std::mutex mutex_;
        std::condition_variable cv_;
        std::queue<T> queue_;
        bool closed_ = false;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_THREAD_SAFE_QUEUE_HPP
