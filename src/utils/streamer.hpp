// streamer.hpp — MJPEG streamer (Linux) with once‑per‑second stats
// ------------------------------------------------------------------
// * Streams the most recent frame at up to `fps` (default 30).
// * Disables Nagle (TCP_NODELAY) for low latency.
// * Prints one concise line per second: pushes‑per‑sec, sends‑per‑sec, average push→send latency.
//   Example:  `[stats] push 15  send 15  lag 7 ms`.
//
#pragma once

#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

class streamer
{
public:
    explicit streamer(uint16_t port = 8081, int fps = 30)
        : port_(port), periodUs_(1'000'000 / std::max(1, fps)),
          lastReport_(std::chrono::steady_clock::now())
    {
        std::cout << "[streamer] start on port " << port_ << ", fps " << fps << '\n';
        srvThread_ = std::thread([this]
                                 { serve(); });
    }

    ~streamer()
    {
        stop_ = true;
        if (srvThread_.joinable())
            srvThread_.join();
        std::cout << "[streamer] stopped\n";
    }

    // Push a raw BGR frame (thread‑safe)
    void push(const cv::Mat &bgr)
    {
        auto now = std::chrono::steady_clock::now();
        lastPushMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());

        std::vector<uchar> jpg;
        cv::imencode(".jpg", bgr, jpg, {cv::IMWRITE_JPEG_QUALITY, 80});
        {
            std::lock_guard<std::mutex> lk(mut_);
            lastJPEG_.swap(jpg);
        }
        pushCount_++;

        // print stats once per second
        if (now - lastReport_ >= std::chrono::seconds(1))
        {
            int pc = pushCount_.exchange(0);
            int sc = sendCount_.exchange(0);
            int lag = 0;
            if (!latencyAcc_.empty())
            {
                lag = std::accumulate(latencyAcc_.begin(), latencyAcc_.end(), 0) / static_cast<int>(latencyAcc_.size());
                latencyAcc_.clear();
            }

            // tempory commented out
            // to avoid too much output in the console
            // std::cout << "[streamer][stats] push=" << pc << "fps, send=" << sc << "fps | lag " << lag << " ms\n";
            lastReport_ = now;
        }
    }

private:
    static bool sendAll(int fd, const void *buf, size_t len)
    {
        const char *p = static_cast<const char *>(buf);
        while (len)
        {
            ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
            if (n <= 0)
                return false;
            p += n;
            len -= n;
        }
        return true;
    }

    void serve()
    {
        int srv = socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{AF_INET, htons(port_), {INADDR_ANY}};
        bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof addr);
        listen(srv, 10);

        while (!stop_)
        {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(srv, &rfds);
            timeval tv{0, 100'000};
            if (select(srv + 1, &rfds, nullptr, nullptr, &tv) > 0)
            {
                int cli = accept(srv, nullptr, nullptr);
                if (cli >= 0)
                    std::thread(&streamer::client, this, cli).detach();
            }
        }
        close(srv);
    }

    // ------------------------------------------------------------------ per‑client loop
    void client(int sock)
    {
        int one = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        char req[1024];
        read(sock, req, sizeof req); // discard HTTP request
        static constexpr char hdr[] =
            "HTTP/1.0 200 OK\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
        if (!sendAll(sock, hdr, sizeof(hdr) - 1))
        {
            close(sock);
            return;
        }

        std::vector<uchar> cached;
        auto lastSent = std::chrono::steady_clock::now() - std::chrono::microseconds(periodUs_);

        while (!stop_)
        {
            {
                std::lock_guard<std::mutex> lk(mut_);
                if (!lastJPEG_.empty())
                    cached = lastJPEG_;
            }
            auto now = std::chrono::steady_clock::now();
            if (now - lastSent >= std::chrono::microseconds(periodUs_))
            {
                if (!cached.empty())
                {
                    std::ostringstream head;
                    head << "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                         << cached.size() << "\r\n\r\n";
                    if (!sendAll(sock, head.str().c_str(), head.str().size()))
                        break;
                    if (!sendAll(sock, cached.data(), cached.size()))
                        break;
                    if (!sendAll(sock, "\r\n", 2))
                        break;

                    sendCount_++;
                    int pushMs = static_cast<int>(lastPushMs_.load());
                    int nowMs = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
                    latencyAcc_.push_back(nowMs - pushMs);
                    lastSent = now;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
        close(sock);
    }

    // ------------------------------------------------------------------ data members
    uint16_t port_;
    int periodUs_;

    std::atomic<bool> stop_{false};
    std::thread srvThread_;

    std::mutex mut_;
    std::vector<uchar> lastJPEG_;

    // stats
    std::atomic<int> pushCount_{0};
    std::atomic<int> sendCount_{0};
    std::vector<int> latencyAcc_;
    std::atomic<long long> lastPushMs_{0};
    std::chrono::steady_clock::time_point lastReport_;
};
