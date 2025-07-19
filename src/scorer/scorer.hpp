#pragma once
#include <string>
#include <vector>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "detector/detector_interface.hpp"
#include "../communication/websocket_service.hpp"
#include "../communication/score_queue.hpp"
#include <memory>

using namespace std;

class Scorer
{
public:
  Scorer(const std::string &model, int width, int height, int fps,
         const std::vector<std::string> &cams, bool debug_mode = false,
         const std::string &detector_type = "geometry");
  ~Scorer();

  void run();
  void stop();

private:
  //  Result sending
  void sendResult(const DetectorResult &result);

  // Configuration
  string model_path;
  int width, height, fps;
  vector<string> camera_sources;
  bool debug_display;
  string detector_type_name;

  // Hardware
  vector<cv::VideoCapture> cameras;
  std::unique_ptr<DetectorInterface> detector;

  // Simple control
  atomic<bool> running{false};

  std::shared_ptr<ScoreQueue> score_queue_;
  std::unique_ptr<WebSocketService> websocket_service_;
};