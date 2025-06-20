#pragma once
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "detector/dart_detector.hpp"

using namespace std;

class Scorer
{
public:
  // Added use_ai parameter to match implementation in cpp file
  Scorer(const std::string &model, int width, int height, int fps,
         const std::vector<std::string> &cams, bool debug_mode = false,
         bool use_ai = false);
  ~Scorer();

  // Start the scoring process and don't return until stopped
  void run();

  // Stop the processing (called from signal handler)
  void stop();

private:
  // The "vision" thread that simulates detection
  void vision_thread_func();

  // Helper methods to make thread synchronization more readable
  void setNewScore(const string &score);
  bool getNewScoreIfAvailable(string &score_out);

  // Simulates dart detection and returns a detected score (or empty string if nothing detected)
  string detect_score();

  // Camera capture related
  vector<cv::VideoCapture> cameras;
  bool initializeCameras();
  vector<cv::Mat> captureFrames();

  // Image preprocessing methods
  std::vector<cv::Mat> preprocessFrames(const std::vector<cv::Mat> &frames);

  // Thread and synchronization
  thread vision_thread;
  atomic<bool> running{false};

  // Shared state between threads - protected by mutex
  mutex score_mutex;
  string current_score{"WAITING"};
  bool new_score_available{false};

  // Configuration (stored but not used in simulation)
  string model_path;
  int width, height, fps;
  vector<string> camera_sources;
  bool debug_display; // Debug mode flag for saving frames

  // Helper to check if a path is a video file
  bool isVideoFile(const string &path);

  // Motion detection
  std::vector<cv::Mat> previous_frames;
  bool detectMotion(const std::vector<cv::Mat> &current_frames);

  // State tracking for dart detection
  enum class DetectionState
  {
    WAITING_FOR_THROW,
    MOTION_DETECTED,
    DART_DETECTED
  };
  DetectionState detection_state = DetectionState::WAITING_FOR_THROW;
  std::chrono::steady_clock::time_point last_motion_time;

  // Detector - using the common interface
  std::unique_ptr<DartDetector> detector;
  bool use_ai_detector = false;

  // Frame averaging for noise reduction
  std::vector<cv::Mat> captureAndAverageFrames(int numFrames = 1);

  // Frame averaging configuration parameters
  struct FrameAveragingParams
  {
    int calibrationFrames = 75; // Number of frames to average for calibration (75 = perfect balance)
    int detectionFrames = 3;    // Number of frames to average for real-time detection
    int retryAttempts = 3;      // Number of calibration retry attempts if capture fails
    int retryDelayMs = 1000;    // Delay between retry attempts in milliseconds
  };

  // Motion detection parameters
  struct MotionDetectionParams
  {
    double motionThresholdRatio = 0.05; // Motion threshold as ratio of frame pixels (5% = 1/20)
    int motionStabilizeMs = 500;        // Wait time after motion stops before detecting dart
  };

  FrameAveragingParams frameParams;
  MotionDetectionParams motionParams;
};