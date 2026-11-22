#ifndef CONCURRENT_QUEUE_H
#define CONCURRENT_QUEUE_H

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <queue>
#include <utility>

namespace bitsim
{
namespace utils::concurrency
{

///
/// \brief A first-in first-out queue that several threads can push to and pop from.
///
/// The thread pool keeps its tasks in one, and the MQTT subscriber the messages that the
/// client's thread receives. pop() waits for an item; pop_for() waits at most a given time,
/// and try_pop() does not wait. close() wakes every thread that waits: they get what is still
/// in the queue, and then std::nullopt. push() on a closed queue drops the item and returns false.
///
template <typename T> class ConcurrentQueue
{
  public:
    typedef T value_type;

    ///
    /// \brief Add an item at the back of the queue
    /// \return false if the queue is closed, and the item was dropped
    ///
    bool push(T value);

    ///
    /// \brief Take the item at the front of the queue, if there is one, without waiting
    ///
    std::optional<T> try_pop();

    ///
    /// \brief Take the item at the front of the queue, waiting at most timeout for one
    /// \return std::nullopt if no item came in time, or the queue is closed and empty
    ///
    template <typename Rep, typename Period>
    std::optional<T> pop_for(std::chrono::duration<Rep, Period> timeout);

    ///
    /// \brief Take the item at the front of the queue, waiting until there is one
    /// \return std::nullopt only once the queue is closed and empty
    ///
    std::optional<T> pop();

    ///
    /// \brief Stop accepting items and wake every thread that waits for one
    ///
    void close();

    ///
    /// \brief Whether close() was called
    ///
    bool is_closed() const;

    ///
    /// \brief The number of items in the queue
    ///
    std::size_t size() const;

    ///
    /// \brief Whether the queue has no items
    ///
    bool empty() const;

  private:
    std::optional<T> take_front_();

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<T> queue_;
    bool closed_{false};
};

template <typename T> bool ConcurrentQueue<T>::push(T value)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_)
        {
            return false;
        }
        queue_.push(std::move(value));
    }
    cv_.notify_one();
    return true;
}

template <typename T> std::optional<T> ConcurrentQueue<T>::take_front_()
{
    if (queue_.empty())
    {
        return std::nullopt;
    }
    std::optional<T> value(std::move(queue_.front()));
    queue_.pop();
    return value;
}

template <typename T> std::optional<T> ConcurrentQueue<T>::try_pop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return take_front_();
}

template <typename T>
template <typename Rep, typename Period>
std::optional<T> ConcurrentQueue<T>::pop_for(std::chrono::duration<Rep, Period> timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, timeout, [this] { return closed_ || !queue_.empty(); });
    return take_front_();
}

template <typename T> std::optional<T> ConcurrentQueue<T>::pop()
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    return take_front_();
}

template <typename T> void ConcurrentQueue<T>::close()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
    }
    cv_.notify_all();
}

template <typename T> bool ConcurrentQueue<T>::is_closed() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
}

template <typename T> std::size_t ConcurrentQueue<T>::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

template <typename T> bool ConcurrentQueue<T>::empty() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

} // namespace utils::concurrency
} // namespace bitsim

#endif // CONCURRENT_QUEUE_H
