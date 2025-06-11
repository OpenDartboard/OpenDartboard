#include "scorer.hpp"
#include "utils/debug.hpp"
#include "detector/detector_factory.hpp" // Added missing include
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
  // Initialize cameras
  initializeCameras();

  // Create the appropriate detector - pass debug flag AND resolution to detector
  detector = DetectorFactory::createDetector(use_ai_detector, debug_display, width, height);

  // Capture initial frames to use for calibration
  vector<Mat> initial_frames = captureFrames();

  // Allow several attempts to get good calibration frames
  for (int attempt = 0; attempt < 3 && initial_frames.empty(); attempt++)
  {
    cout << "Retrying to capture initial frames, attempt " << attempt + 1 << endl;
    this_thread::sleep_for(chrono::milliseconds(500));
    initial_frames = captureFrames();
  }

  if (initial_frames.empty())
  {
    cerr << "Warning: Could not capture initial frames for calibration" << endl;

    // Basic initialization without frames
    if (!detector->initialize(model_path))
    {
      cerr << "Warning: Failed to initialize detector. Using simulation mode." << endl;
    }
    else
    {
      cout << "Detector initialized without calibration - will calibrate on first detection" << endl;
    }
  }
  else
  {
    // Initialize with frames for immediate calibration
    cout << "Calibrating detector with initial frames..." << endl;

    // We need to cast to the specific detector type
    // This is a workaround - normally we'd enhance the interface
    if (GeometryDetector *geo_detector = dynamic_cast<GeometryDetector *>(detector.get()))
    {
      if (geo_detector->initialize(model_path, initial_frames))
      {
        cout << "Detector calibrated and initialized successfully" << endl;
      }
      else
      {
        cerr << "Warning: Detector initialization with calibration failed" << endl;
      }
    }
    else
    {
      // Fallback to normal initialization for other detector types
      if (!detector->initialize(model_path))
      {
        cerr << "Warning: Failed to initialize detector. Using simulation mode." << endl;
      }
      else
      {
        cout << "Detector initialized successfully" << endl;
      }
    }
  }
}

// Re-adding the destructor
Scorer::~Scorer()
{
  stop();
}

// Determine if a given path is a video file based on its extension
// This is a simple check based on common video file extensions
bool Scorer::isVideoFile(const string &path)
{
  // Convert to lowercase for case-insensitive comparison
  string lower_path = path;
  transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);

  // Check common video extensions
  const vector<string> video_extensions = {".mp4", ".avi", ".mkv", ".mov", ".wmv"};
  for (const auto &ext : video_extensions)
  {
    if (lower_path.length() >= ext.length() &&
        lower_path.substr(lower_path.length() - ext.length()) == ext)
    {
      return true;
    }
  }
  return false;
}

// Initialize cameras based on provided sources
// This method handles both device paths and video files
// It also sets the desired width, height, and fps for each camera
bool Scorer::initializeCameras()
{
  cameras.clear();

  cout << "Initializing " << camera_sources.size() << " cameras..." << endl;

  for (size_t i = 0; i < camera_sources.size(); i++)
  {
    VideoCapture cap;

    // Try to open the camera
    cout << "Opening camera " << i + 1 << ": " << camera_sources[i] << endl;

    // Check if this is a video file
    if (isVideoFile(camera_sources[i]))
    {
      cout << "Detected video file: " << camera_sources[i] << endl;
      cap.open(camera_sources[i]);
    }
    else
    {
      // Handle both device paths and numeric indices
      cap.open(camera_sources[i]);
    }

    if (!cap.isOpened())
    {
      cerr << "Failed to open camera/video " << camera_sources[i] << endl;
      return false;
    }

    // Always set dimensions and fps - for video files, this tells OpenCV
    // what dimensions we want the frames to be (if the backend supports it)
    cap.set(CAP_PROP_FRAME_WIDTH, width);
    cap.set(CAP_PROP_FRAME_HEIGHT, height);
    cap.set(CAP_PROP_FPS, fps);

    // Add to our camera array
    cameras.push_back(cap);

    cout << "Camera/video " << i + 1 << " initialized successfully" << endl;
  }

  return cameras.size() > 0;
}

// We still need to handle resizing in captureFrames() since setting properties
// in initializeCameras() might not work for all video backends
vector<Mat> Scorer::captureFrames()
{
  vector<Mat> frames;

  for (size_t i = 0; i < cameras.size(); i++)
  {
    Mat frame;
    bool success = cameras[i].read(frame);

    if (!success || frame.empty())
    {
      cerr << "Failed to capture frame from camera " << i + 1 << endl;
      continue;
    }

    // Check if we need to resize the frame
    if (!frame.empty() && (frame.cols != width || frame.rows != height))
    {
      Mat resized;
      resize(frame, resized, Size(width, height));
      frames.push_back(resized);
    }
    else
    {
      frames.push_back(frame);
    }
  }

  return frames;
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
      cerr << "Empty frame detected in camera " << i + 1 << endl;
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
    int motion_threshold = (motion_mask.rows * motion_mask.cols) / 20; // 5% of frame

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
  // Step 1: Capture frames from all cameras
  vector<Mat> frames = captureFrames();

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
    if (!motion && chrono::duration_cast<chrono::milliseconds>(now - last_motion_time).count() > 500)
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
  cout << "Vision thread started" << endl;

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

  cout << "Vision thread stopped" << endl;
}

void Scorer::run()
{
  // Set the running flag
  running = true;

  // Start the vision thread that will simulate detections
  vision_thread = thread(&Scorer::vision_thread_func, this);

  // Print startup message
  cout << "Scorer running with " << camera_sources.size() << " cameras" << endl;

  // Main loop - check for and print new scores
  while (running)
  {
    string new_score;

    // Check for new scores using our helper method
    if (getNewScoreIfAvailable(new_score))
    {
      cout << new_score << endl;
    }

    // Small sleep to avoid busy waiting - increase to 100ms
    // Human perception is ~10Hz, so 100ms is still very responsive
    // while reducing CPU usage compared to 50ms
    this_thread::sleep_for(chrono::milliseconds(100));
  }
}