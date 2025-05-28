#pragma once
#include <string>
#include <vector>

class Scorer
{
public:
  Scorer(const std::string &model,
         int width, int height, int fps,
         const std::vector<std::string> &cams);

  // In the real implementation this runs an ncnn forward pass
  std::string get_prediction();

private:
  // store config, ncnn::Net, etc.
};