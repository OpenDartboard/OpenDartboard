#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <cstdint>
#include <iostream>
#include "utils.hpp"
#include "detector/geometry/calibration/geometry_calibration.hpp"

using namespace cv;
using namespace std;

namespace cache
{
    namespace geometry
    {

        static const uint32_t MAGIC = 0x42554C4C; // "BULL" in ASCII
        static const uint32_t VERSION = 1;

        struct FileHeader
        {
            uint32_t magic = MAGIC;     // File signature
            uint32_t version = VERSION; // Version for future compatibility
            uint32_t count = 0;         // Number of calibrations
        };

        // Generate standard filename for calibration cache
        inline string generateFilename()
        {
            // Create cache directory if it doesn't exist
            system("mkdir -p cache");
            return "cache/geometry_calibration.dat";
        }

        // Load calibration data from binary file - return calibrations directly
        inline vector<DartboardCalibration> load()
        {
            string filename = generateFilename();
            try
            {
                ifstream file(filename, ios::binary);
                if (!file)
                {
                    log_debug("No calibration file found will create new one: " + filename);
                    return {}; // Return empty vector
                }

                // Read and verify header
                FileHeader header;
                file.read((char *)&header, sizeof(header));

                if (header.magic != MAGIC)
                {
                    log_warning("Invalid calibration file format");
                    return {};
                }

                if (header.version != VERSION)
                {
                    log_warning("Incompatible calibration file version: " + to_string(header.version) + ". Expected " + to_string(VERSION));
                    return {};
                }

                // Read calibration data
                vector<DartboardCalibration> calibrations(header.count);
                file.read((char *)calibrations.data(), sizeof(DartboardCalibration) * header.count);

                if (!file.good())
                {
                    log_error("Failed to read calibration data");
                    return {};
                }

                log_info("Loaded " + to_string(header.count) + " calibrations from cache");
                return calibrations;
            }
            catch (const exception &e)
            {
                log_error("Error loading calibration: " + string(e.what()));
                return {}; // Return empty vector on error
            }
        }

        // Save calibration data to binary file - calibrations only
        inline bool save(const vector<DartboardCalibration> &calibrations)
        {
            string filename = generateFilename();
            try
            {
                ofstream file(filename, ios::binary);
                if (!file)
                {
                    log_error("Failed to open file for writing: " + filename);
                    return false;
                }

                // Write header
                FileHeader header;
                header.count = (uint32_t)calibrations.size();
                file.write((char *)&header, sizeof(header));

                // Write calibration data
                file.write((char *)calibrations.data(), sizeof(DartboardCalibration) * calibrations.size());

                if (!file.good())
                {
                    log_error("Failed to write calibration data");
                    return false;
                }

                log_info("Saved " + to_string(calibrations.size()) + " calibrations to cache");
                return true;
            }
            catch (const exception &e)
            {
                log_error("Error saving calibration: " + string(e.what()));
                return false;
            }
        }

        // Save background frames as individual image files
        inline bool saveBackgroundFrames(const vector<Mat> &background_frames)
        {
            try
            {
                system("mkdir -p cache/backgrounds");

                for (size_t i = 0; i < background_frames.size(); i++)
                {
                    if (!background_frames[i].empty())
                    {
                        string filename = "cache/backgrounds/camera_" + to_string(i) + "_background.jpg";
                        if (!imwrite(filename, background_frames[i]))
                        {
                            log_error("Failed to save background frame for camera " + to_string(i));
                            return false;
                        }
                    }
                }

                log_info("Saved " + to_string(background_frames.size()) + " background frames");
                return true;
            }
            catch (const exception &e)
            {
                log_error("Error saving background frames: " + string(e.what()));
                return false;
            }
        }

        // Load background frames from individual image files
        inline vector<Mat> loadBackgroundFrames()
        {
            vector<Mat> background_frames;

            try
            {
                // Try to load background frames (check for up to 3 cameras)
                for (int i = 0; i < 3; i++)
                {
                    string filename = "cache/backgrounds/camera_" + to_string(i) + "_background.jpg";
                    Mat frame = imread(filename);

                    if (!frame.empty())
                    {
                        background_frames.push_back(frame);
                        log_debug("Loaded background frame for camera " + to_string(i));
                    }
                    else
                    {
                        // Stop when we can't find more consecutive frames
                        break;
                    }
                }

                if (!background_frames.empty())
                {
                    log_info("Loaded " + to_string(background_frames.size()) + " background frames");
                }

                return background_frames;
            }
            catch (const exception &e)
            {
                log_error("Error loading background frames: " + string(e.what()));
                return vector<Mat>();
            }
        }
    }
}