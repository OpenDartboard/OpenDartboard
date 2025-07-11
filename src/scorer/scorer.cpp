#include "scorer.hpp"
#include "utils.hpp"
#include "detector/detector_factory.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <opencv2/opencv.hpp>
#include <algorithm>

using namespace std;
using namespace cv;

Scorer::Scorer(const string &model, int w, int h, int fps, const vector<string> &cams, bool debug_mode, const string &detector_type)
    : model_path(model), width(w), height(h), fps(fps), camera_sources(cams), debug_display(debug_mode), detector_type_name(detector_type)
{
  // Initialize cameras
  if (!camera::initializeCameras(cameras, camera_sources, width, height, fps))
  {
    log_error("Failed to initialize cameras");
    return;
  }

  // Create detector
  detector = DetectorFactory::createDetector(detector_type_name, debug_display, width, height, fps);

  // Initialize detector
  if (!detector->initialize(cameras))
  {
    log_error("Failed to initialize detector.");
  }
  else
  {
    log_info("Detector initialized successfully");
  }
}

Scorer::~Scorer()
{
  stop();
}

void Scorer::stop()
{
  running = false;
}

void Scorer::sendResult(const DetectorResult &result)
{
  // For now, just log it - later this becomes WebSocket/API
  if (result.dart_detected)
  {
    log_info("SCORE: " + result.score +
             " | Position: (" + to_string((int)result.position.x) + "," + to_string((int)result.position.y) + ")" +
             " | Confidence: " + to_string(result.confidence) +
             " | Camera: " + to_string(result.camera_index) +
             " | Processing: " + to_string(result.processing_time_ms) + "ms");
  }

  if (debug_display && result.motion_detected)
  {
    log_debug("Motion detected on frame");
  }
}

void Scorer::run()
{
  if (!detector->isInitialized())
  {
    log_error("Detector not initialized - cannot run");
    return;
  }

  running = true;
  log_info("Scorer running with " + to_string(camera_sources.size()) + " cameras");
  log_info("Using detector: " + detector_type_name);

  while (running)
  {
    // 1. Capture frames
    vector<Mat> frames = camera::captureFrames(cameras, width, height);

    if (frames.empty())
    {
      log_warning("No frames captured, retrying...");
      this_thread::sleep_for(chrono::milliseconds(100));
      continue;
    }

    // 2. Process frames (detector handles motion + detection internally)
    DetectorResult result = detector->process(frames);

    // 3. Send result if something detected
    if (result)
    { // Uses implicit bool conversion
      sendResult(result);
    }

    // 4. Frame rate control (~15 FPS)
    this_thread::sleep_for(chrono::milliseconds(66));
  }

  log_info("Scorer stopped");
}