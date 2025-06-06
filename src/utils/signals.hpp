#pragma once
#include <csignal>
#include <iostream>
#include <functional>

namespace signals
{

    // Global callback for signal handling
    static std::function<void()> shutdownCallback;

    // Signal handler function
    inline void signalHandler(int signal)
    {
        std::cout << "Received signal " << signal << ", shutting down..." << std::endl;

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
        std::cout << "Signal handlers registered for graceful shutdown" << std::endl;
    }

} // namespace signals
