#pragma once
#include <csignal>
#include <functional>
#include "logging.hpp"

namespace signals
{

    // Global callback for signal handling
    static std::function<void()> shutdownCallback;

    // Signal handler function
    inline void signalHandler(int signal)
    {
        log_warning("Received signal " + std::to_string(signal) + ", shutting down...");

        // Call the registered shutdown callback if it exists
        if (shutdownCallback)
        {
            shutdownCallback();
        }

        exit(signal);
    }

    // Register signal handlers and shutdown callback
    inline void setupSignalHandlers(std::function<void()> callback)
    {
        shutdownCallback = callback;
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        log_info("Signal handlers registered for graceful shutdown");
    }

} // namespace signals
