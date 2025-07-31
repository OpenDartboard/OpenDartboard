// utils/autocam.hpp  (updated)
#pragma once
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <map>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <cstdio>

namespace autocam
{
    inline bool isVideoCapture(const std::string &dev)
    {
        int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0)
            return false;

        v4l2_capability cap{};
        bool ok = (::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) &&
                  (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) &&
                  (cap.device_caps & V4L2_CAP_STREAMING);
        ::close(fd);
        return ok;
    }

    inline std::vector<std::tuple<int, int, int>>
    listMjpgModes(int fd)
    {
        std::vector<std::tuple<int, int, int>> modes;

        v4l2_fmtdesc fmtDesc{};
        fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while (::ioctl(fd, VIDIOC_ENUM_FMT, &fmtDesc) == 0)
        {
            if (fmtDesc.pixelformat == V4L2_PIX_FMT_MJPEG)
            {
                v4l2_frmsizeenum fs{};
                fs.pixel_format = fmtDesc.pixelformat;
                while (::ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs) == 0)
                {
                    if (fs.type == V4L2_FRMSIZE_TYPE_DISCRETE)
                    {
                        int w = fs.discrete.width;
                        int h = fs.discrete.height;

                        v4l2_frmivalenum fi{};
                        fi.pixel_format = fmtDesc.pixelformat;
                        fi.width = w;
                        fi.height = h;
                        while (::ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fi) == 0)
                        {
                            if (fi.type == V4L2_FRMIVAL_TYPE_DISCRETE)
                            {
                                int fps = fi.discrete.denominator / fi.discrete.numerator;
                                modes.emplace_back(w, h, fps);
                            }
                            fi.index++;
                        }
                    }
                    fs.index++;
                }
            }
            fmtDesc.index++;
        }
        return modes;
    }

    inline bool supportsMjpg(int fd, int w, int h, int fps)
    {
        auto modes = listMjpgModes(fd);
        for (auto &m : modes)
            if (std::get<0>(m) == w && std::get<1>(m) == h && std::get<2>(m) == fps)
                return true;
        return false;
    }

    inline bool configureCam(const std::string &dev, int w, int h, int fps, bool verbose)
    {
        int fd = ::open(dev.c_str(), O_RDWR);
        if (fd < 0)
            return false;

        if (!supportsMjpg(fd, w, h, fps))
        {
            if (verbose)
            {
                std::fprintf(stderr,
                             "[autocam] %s does NOT support %dx%d@%d MJPG. Supported modes:\n",
                             dev.c_str(), w, h, fps);
                for (auto &m : listMjpgModes(fd))
                    std::fprintf(stderr, "  %dx%d @ %d FPS\n",
                                 std::get<0>(m), std::get<1>(m), std::get<2>(m));
            }
            ::close(fd);
            return false;
        }

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = w;
        fmt.fmt.pix.height = h;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (::ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
        {
            ::close(fd);
            return false;
        }

        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe = {1, static_cast<unsigned int>(fps)};
        ::ioctl(fd, VIDIOC_S_PARM, &parm);
        ::close(fd);
        return true;
    }

    inline std::tuple<int, int, int> getActualSettings(const std::string &dev)
    {
        int fd = ::open(dev.c_str(), O_RDWR);
        if (fd < 0)
            return {0, 0, 0};

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (::ioctl(fd, VIDIOC_G_FMT, &fmt) < 0)
        {
            ::close(fd);
            return {0, 0, 0};
        }

        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ::ioctl(fd, VIDIOC_G_PARM, &parm);

        int actualFps = parm.parm.capture.timeperframe.denominator /
                        parm.parm.capture.timeperframe.numerator;

        ::close(fd);
        return {fmt.fmt.pix.width, fmt.fmt.pix.height, actualFps};
    }

    inline std::vector<std::string>
    detectAndLock(int maxCams, int width, int height, int fps, bool verbose = true)
    {
        std::vector<std::string> found;
        DIR *d = ::opendir("/dev");
        if (!d)
            throw std::runtime_error("opendir /dev failed");

        if (verbose)
            std::fprintf(stderr, "Scanning /dev for video devices...\n");

        struct dirent *e;
        while ((e = ::readdir(d)))
        {
            if (std::strncmp(e->d_name, "video", 5) == 0)
            {
                std::string dev = "/dev/" + std::string(e->d_name);

                if (!isVideoCapture(dev))
                {
                    continue;
                }
                if (configureCam(dev, width, height, fps, verbose))
                {
                    found.push_back(dev);
                    if ((int)found.size() == maxCams)
                        break;
                }
            }
        }
        ::closedir(d);
        std::sort(found.begin(), found.end());

        if (verbose)
        {
            std::fprintf(stderr, "Found %d compatible camera(s) with actual settings:\n", (int)found.size());
            for (const auto &cam : found)
            {
                auto [w, h, fps] = getActualSettings(cam);
                std::fprintf(stderr, " ─ %s: %dx%d @ %d FPS\n", cam.c_str(), w, h, fps);

                // Show supported modes grouped by resolution
                int fd = ::open(cam.c_str(), O_RDWR);
                if (fd >= 0)
                {
                    auto modes = listMjpgModes(fd);
                    std::map<std::pair<int, int>, std::vector<int>> resolutionMap;

                    // Group fps by resolution
                    for (const auto &mode : modes)
                    {
                        std::pair<int, int> res = {std::get<0>(mode), std::get<1>(mode)};
                        resolutionMap[res].push_back(std::get<2>(mode));
                    }

                    // Print grouped modes
                    for (const auto &[resolution, fpsValues] : resolutionMap)
                    {
                        std::fprintf(stderr, " ─── [%dx%d @ ", resolution.first, resolution.second);
                        for (size_t i = 0; i < fpsValues.size(); ++i)
                        {
                            if (i > 0)
                                std::fprintf(stderr, "/");
                            std::fprintf(stderr, "%dfps", fpsValues[i]);
                        }
                        std::fprintf(stderr, "]\n");
                    }
                    ::close(fd);
                }
            }
        }

        if ((int)found.size() < maxCams)
            throw std::runtime_error("[autocam] only " + std::to_string(found.size()) +
                                     " fully-compatible cameras detected; need " + std::to_string(maxCams));
        return found;
    }

} // namespace autocam