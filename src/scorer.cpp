#include "scorer.hpp"

// In a real file you’d #include <ncnn/net.h> and build pipelines.

Scorer::Scorer(const std::string & /*model*/,
               int /*w*/, int /*h*/, int /*fps*/,
               const std::vector<std::string> & /*cams*/)
{
  // TODO: load ncnn::Net, open GStreamer pipelines
}

std::string Scorer::get_prediction()
{
  // TODO: read frame(s) and run inference
  return "MISS"; // hard-coded stub so you can see the installer working
}