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

Scorer::Scorer(const string &model, int w, int h, int fps, const vector<string> &cams, bool debug_mode, bool use_ai)
    : model_path(model), width(w), height(h), fps(fps), camera_sources(cams), debug_display(debug_mode), use_ai_detector(use_ai)
{
  // Initialize cameras using utility function
  if (!camera::initializeCameras(cameras, camera_sources, width, height, fps))
  {
    log_error("Failed to initialize cameras");
    return;
  }

  // Create the appropriate detector - pass debug flag AND resolution to detector
  detector = DetectorFactory::createDetector(use_ai_detector, debug_display, width, height, fps);

  // Simple initialization - let detector handle its own calibration needs
  if (!detector->initialize(cameras))
  {
    log_error("Failed to initialize detector.");
  }
  else
  {
    log_info("Detector initialized successfully");
  }
}

// Re-adding the destructor
Scorer::~Scorer()
{
  stop();
}

// The main run method that starts the vision thread
void Scorer::stop()
{
  running = false;
  if (vision_thread.joinable())
  {
    vision_thread.join();
  }
}

// Helper method to set a new score (handles the mutex locking internally)
void Scorer::setNewScore(const string &score)
{
  lock_guard<mutex> lock(score_mutex);
  current_score = score;
  new_score_available = true;
}

// Helper method to get a new score if one is available
bool Scorer::getNewScoreIfAvailable(string &score_out)
{
  lock_guard<mutex> lock(score_mutex);
  if (new_score_available)
  {
    score_out = current_score;
    new_score_available = false;
    return true;
  }
  return false;
}

// Detect motion between frames
bool Scorer::detectMotion(const vector<Mat> &current_frames)
{
  // Skip if we don't have previous frames for comparison
  if (previous_frames.empty() || previous_frames.size() != current_frames.size())
  {
    // Store current frames as previous for next comparison
    previous_frames.clear();
    for (const auto &frame : current_frames)
    {
      previous_frames.push_back(frame.clone());
    }
    return false;
  }

  bool motion_detected = false;

  // Check each camera for motion
  for (size_t i = 0; i < current_frames.size() && i < previous_frames.size(); i++)
  {
    // Safety check for empty frames
    if (current_frames[i].empty() || previous_frames[i].empty())
    {
      log_error("Empty frame detected in camera " + log_string(i + 1));
      continue;
    }

    // Convert frames to grayscale
    Mat prev_gray = debug::convertToGrayscale(previous_frames[i]);
    Mat curr_gray = debug::convertToGrayscale(current_frames[i]);

    // Get thresholded motion image
    Mat motion_mask = debug::applyMotionThreshold(prev_gray, curr_gray);

    // Skip if motion_mask is empty (error occurred)
    if (motion_mask.empty())
    {
      continue;
    }

    // Count non-zero pixels to determine amount of motion
    int motion_pixels = countNonZero(motion_mask);
    int motion_threshold = (motion_mask.rows * motion_mask.cols) * motionParams.motionThresholdRatio;

    if (motion_pixels > motion_threshold)
    {
      motion_detected = true;

      // Save motion frames in debug mode
      if (debug_display)
      {
        debug::saveFrames({motion_mask}, "motion");
      }
      break;
    }
  }

  // Update previous frames for next comparison
  previous_frames.clear();
  for (const auto &frame : current_frames)
  {
    previous_frames.push_back(frame.clone());
  }

  return motion_detected;
}

string Scorer::detect_score()
{
  // Step 1: Capture frames from all cameras (use averaging for better quality)
  vector<Mat> frames = camera::captureAndAverageFrames(cameras, width, height, fps, frameParams.detectionFrames);

  if (frames.empty())
  {
    cerr << "No frames captured from any camera" << endl;
    return "";
  }

  // Save raw frames if debug mode is enabled
  if (debug_display)
  {
    // temporarily disabled to avoid excessive file writes
    // debug::saveFrames(frames);
  }

  // Step 2: Detect motion between frames
  bool motion = detectMotion(frames);

  // State machine for dart detection
  auto now = chrono::steady_clock::now();

  switch (detection_state)
  {
  case DetectionState::WAITING_FOR_THROW:
    if (motion)
    {
      detection_state = DetectionState::MOTION_DETECTED;
      last_motion_time = now;
    }
    break;

  case DetectionState::MOTION_DETECTED:
    // If motion has stopped for a certain period, we might have a dart
    if (!motion && chrono::duration_cast<chrono::milliseconds>(now - last_motion_time).count() > motionParams.motionStabilizeMs)
    {
      detection_state = DetectionState::DART_DETECTED;

      // Check if we should stop
      if (!running)
        return "";

      // Pass raw frames to detector and get score
      string score = "MISS"; // Default to miss

      if (detector && detector->isInitialized())
      {
        // Let the detector handle everything else
        std::vector<DartDetection> detections = detector->detectDarts(frames);

        // Use the detector's score if available
        if (!detections.empty() && !detections[0].score.empty())
        {
          score = detections[0].score;
        }
      }

      // Reset for next throw
      detection_state = DetectionState::WAITING_FOR_THROW;
      return score;
    }
    else if (motion)
    {
      // Update the last motion time if we're still seeing motion
      last_motion_time = now;
    }
    break;

  case DetectionState::DART_DETECTED:
    if (motion)
    {
      detection_state = DetectionState::WAITING_FOR_THROW;
      return "END";
    }
    break;
  }

  return ""; // No score to report yet
}

// This simulates the computer vision thread that would detect darts
void Scorer::vision_thread_func()
{
  log_info("Vision thread started");

  while (running)
  {
    // Try to detect a dart score (this now includes real camera capture)
    string score = detect_score();

    // If we got a score, update it
    if (!score.empty())
    {
      setNewScore(score);
    }

    // Small delay to control frame rate
    this_thread::sleep_for(chrono::milliseconds(1000 / fps));
  }

  log_info("Vision thread stopped");
}

void Scorer::run()
{

  if (detector->isInitialized() == false)
  {
    log_error("Detector is not initialized. Cannot run scorer. Please check your camera setup.");

    int attempts = 0;      // Try to wait for cameras to be set up
    int max_attempts = 50; // Try for 50 attempts

    while (attempts < max_attempts)
    {

      log_warning("Will attempt to initialize again in 1 minutes... Attempt " + log_string(attempts + 1) + "/" + log_string(max_attempts));
      this_thread::sleep_for(chrono::seconds(60)); // Wait 1 minutes between attempts

      // try to reinitialize the detector
      if (detector->initialize(cameras))
      {
        break;
      }

      attempts++;
    }

    if (attempts >= max_attempts)
    {
      log_error("Failed to initialize detector after " + log_string(max_attempts) + " attempts. Exiting.");
      return; // Exit if we can't initialize
    }
  }

  // Set the running flag
  running = true;

  // Start the vision thread that will simulate detections
  vision_thread = thread(&Scorer::vision_thread_func, this);

  // Print startup message
  log_info("Scorer running with " + log_string(camera_sources.size()) + " cameras");

  // Main loop - check for and print new scores
  while (running)
  {
    string new_score;

    // Check for new scores using our helper method
    if (getNewScoreIfAvailable(new_score))
    {
      log_info(new_score);
    }

    // Small sleep to avoid busy waiting - increase to 100ms
    // Human perception is ~10Hz, so 100ms is still very responsive
    // while reducing CPU usage compared to 50ms
    this_thread::sleep_for(chrono::milliseconds(100));
  }
}