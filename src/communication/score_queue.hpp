#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include "detector/detector_interface.hpp"

class ScoreQueue
{
public:
    void push(const DetectorResult &result);
    bool pop(DetectorResult &result, int timeout_ms = 100);
    size_t size() const;

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::queue<DetectorResult> queue_;
};
