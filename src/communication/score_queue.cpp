#include "score_queue.hpp"

void ScoreQueue::push(const DetectorResult &result)
{
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(result);
    condition_.notify_one();
}

bool ScoreQueue::pop(DetectorResult &result, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (condition_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                            [this]
                            { return !queue_.empty(); }))
    {
        result = queue_.front();
        queue_.pop();
        return true;
    }
    return false;
}

size_t ScoreQueue::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}
