#include "scorer/scorer.hpp"
#include "utils/args.hpp"
#include "utils/debug.hpp"
#include "utils/signals.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

using namespace std;

// version string for the application
const string version = "0.1.0"; // Update this as needed

int main(int argc, char **argv)
{
  // Check for help or version flags first
  if (hasFlag(argc, argv, "--version"))
    debug::printVersionAndExit(version);
  if (hasFlag(argc, argv, "--help"))
    debug::printHelpAndExit();

  // Parse command line arguments with defaults
  string model_path = getArg(argc, argv, "--model", "/usr/local/share/opendartboard/models/dart.param");
  vector<string> cams = getArgVector(argc, argv, "--cams", "/dev/video0,/dev/video1,/dev/video2");
  int width = getArg(argc, argv, "--width", 640);
  int height = getArg(argc, argv, "--height", 480);
  int fps = getArg(argc, argv, "--fps", 15);

  // Print startup and configuration information
  debug::printStartup("OpenDartboard", version);
  debug::printConfig(width, height, fps, model_path, cams);

  // Initialise the scorer
  Scorer scorer(model_path, width, height, fps, cams);

  // Register signal handlers with a lambda to stop the scorer
  signals::setupSignalHandlers([&scorer]()
                               { scorer.stop(); });

  // Start the scorer processing in background thread
  scorer.run();

  // Best practice: wait for the scorer thread to finish
  return 0;
}