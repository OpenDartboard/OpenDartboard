#include "scorer.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
  // Hard-code the three cams for now
  std::vector<std::string> cams{"/dev/video0", "/dev/video1", "/dev/video2"};
  std::string model_path{"/usr/local/models/dart.param"}; // param; .bin is auto-loaded
  int width = 640;
  int height = 480;
  int fps = 15;

  // Initialise the scorer (this does nothing yet)
  Scorer scorer(model_path, width, height, fps, cams);

  // Main loop – for demo just print 1 token every second
  while (true)
  {
    std::string result = scorer.get_prediction(); // returns "MISS" placeholder
    std::cout << result << std::endl;             // stdout protocol
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}