#pragma once
#include "score_queue.hpp"
#include <thread>
#include <atomic>
#include <memory>
#include <httplib.h>

class WebSocketService
{
public:
    WebSocketService(std::shared_ptr<ScoreQueue> queue, int port = 13520);
    ~WebSocketService();

    void start();
    void stop();
    bool isRunning() const { return running_; }

private:
    void run();
    void broadcastScore(const std::string &json_message); // Updated signature
    std::string formatScoreJson(const DetectorResult &result);

    std::shared_ptr<ScoreQueue> score_queue_;
    std::thread worker_thread_;
    std::atomic<bool> running_{false};
    std::unique_ptr<httplib::Server> server_; // Use httplib, not libwebsockets
    int port_;
};
