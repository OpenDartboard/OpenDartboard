#include "scorer.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <random>

using namespace std;

Scorer::Scorer(const string &model, int w, int h, int fps, const vector<string> &cams)
    : model_path(model), width(w), height(h), fps(fps), camera_sources(cams)
{
  // Just store configuration, no simulation setup
}

Scorer::~Scorer()
{
  stop();
}

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

string Scorer::detect_score()
{
  // All simulation code is contained in this function

  // Setup the random number generator
  static random_device rd;
  static mt19937 gen(rd());

  // Define possible outputs (the fixed sequence we want)
  static const vector<string> scores = {"15", "MISS", "20", "END"};

  // Define distributions
  static uniform_int_distribution<int> score_dist(0, scores.size() - 1);
  static uniform_int_distribution<int> time_dist(1000, 3000);

  // Simulate the time it takes to process frames and detect a dart
  this_thread::sleep_for(chrono::milliseconds(time_dist(gen)));

  // Check if we should stop
  if (!running)
    return "";

  // In a real implementation, this would:
  // 1. Grab frames from cameras
  // 2. Process frames with computer vision
  // 3. Run inference on detected dart
  // 4. Return the score or empty string if no dart detected

  // For simulation, just return a random score
  return scores[score_dist(gen)];
}

// This simulates the computer vision thread that would detect darts
void Scorer::vision_thread_func()
{
  cout << "Vision thread started" << endl;

  // Clean main loop that just detects and updates
  while (running)
  {
    // Try to detect a dart score
    string score = detect_score();

    // If we got a score, update it
    if (!score.empty())
    {
      setNewScore(score);
    }
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

    // Small sleep to avoid busy waiting
    this_thread::sleep_for(chrono::milliseconds(50));
  }
}