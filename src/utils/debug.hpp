#pragma once
#include <iostream>
#include <string>
#include <vector>

namespace debug
{

    // Print application startup banner
    inline void printStartup(const std::string &appName, const std::string &version)
    {
        std::cout << "=====================================\n";
        std::cout << "  " << appName << " v" << version << " starting...\n";
        std::cout << "=====================================\n";
    }

    // Print configuration details
    inline void printConfig(int width, int height, int fps,
                            const std::string &model,
                            const std::vector<std::string> &cams)
    {
        std::cout << "Configuration:\n";
        std::cout << "  - Resolution: " << width << "x" << height << "\n";
        std::cout << "  - FPS: " << fps << "\n";
        std::cout << "  - Model: " << model << "\n";
        std::cout << "  - Cameras (" << cams.size() << "):\n";
        for (size_t i = 0; i < cams.size(); i++)
        {
            std::cout << "      " << (i + 1) << ": " << cams[i] << "\n";
        }
        std::cout << "-------------------------------------\n";
    }

    // Print version information and exit
    inline void printVersionAndExit(const std::string &version)
    {
        std::cout << "OpenDartboard runtime version: " << version << std::endl;
        exit(0);
    }

    // Print help message and exit
    inline void printHelpAndExit()
    {
        std::cout << "Usage: opendartboard [options]\n";
        std::cout << "Options:\n";
        std::cout << "  --model <path>     Path to the model file (default: /usr/local/share/opendartboard/models/dart.param)\n";
        std::cout << "  --cams <cameras>   Comma-separated list of camera devices (default: /dev/video0,/dev/video1,/dev/video2)\n";
        std::cout << "  --width <width>    Frame width (default: 640)\n";
        std::cout << "  --height <height>  Frame height (default: 480)\n";
        std::cout << "  --fps <fps>        Frames per second (default: 15)\n";
        std::cout << "  --version          Show version information\n";
        std::cout << "  --help             Show this help message\n";
        exit(0);
    }

} // namespace debug
