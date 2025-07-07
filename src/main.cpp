#include "scorer/scorer.hpp"
#include "utils/args.hpp"
#include "utils/debug.hpp"
#include "utils/signals.hpp"
#include "utils/logging.hpp"
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
  int height = getArg(argc, argv, "--height", 360);
  int fps = getArg(argc, argv, "--fps", 15);
  bool debug_mode = hasFlag(argc, argv, "--debug") || hasFlag(argc, argv, "-d");
  bool quite_mode = hasFlag(argc, argv, "--quiet") || hasFlag(argc, argv, "-q");

  // Replace boolean flag with detector type string
  string detector_type = getArg(argc, argv, "--detector", "geometry");

  // set log level based on debug mode
  if (debug_mode)
  {
    logging::setLogLevel(logging::LogLevel::DEBUG); // Show everything
    log_info("Debug mode enabled - showing all log messages");
  }
  else if (quite_mode)
  {
    logging::setLogLevel(logging::LogLevel::ERROR); // Only errors
    log_info("Quiet mode enabled - showing only error messages");
  }

  // Print startup and configuration information
  debug::printStartup("OpenDartboard", version);
  debug::printConfig(width, height, fps, model_path, cams);

  // Initialise the scorer with debug mode if requested
  Scorer scorer(model_path, width, height, fps, cams, debug_mode, detector_type);

  // Register signal handlers with a lambda to stop the scorer
  signals::setupSignalHandlers([&scorer]()
                               { scorer.stop(); });

  // Start the scorer processing in background thread
  scorer.run();

  // Best practice: wait for the scorer thread to finish
  return 0;
}