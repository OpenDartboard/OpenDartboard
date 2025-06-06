#pragma once
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

using namespace std;

class Scorer
{
public:
  Scorer(const std::string &model, int width, int height, int fps, const std::vector<std::string> &cams);
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
};