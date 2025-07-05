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
            return "geometry_calibration.dat";
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
    }
}