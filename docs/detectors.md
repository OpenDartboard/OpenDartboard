# OpenDartboard Detector Plugins

OpenDartboard supports multiple detector types through a flexible plugin system. You can use built-in detectors or create custom ones.

## Usage

### Built-in Detectors

```bash
# Use geometry detector (default)
opendartboard --detector=geometry

# Use AI detector (fallback to geometry for now)
opendartboard --detector=ai
```

### Custom Detectors

```bash
# Load custom detector plugin
opendartboard --detector=myCoolDetector

# This looks for:
# 1. detectors/myCoolDetector/libmyCoolDetector.so
# 2. detectors/libmyCoolDetector.so
# 3. detectors/myCoolDetector.so
```

## Creating Custom Detectors

### 1. Implement the Interface

```cpp
// myCoolDetector.hpp
#pragma once
#include "detector_interface.hpp"

class MyCoolDetector : public DetectorInterface {
public:
    MyCoolDetector(bool debug_mode, int width, int height, int fps);
    virtual ~MyCoolDetector() = default;

    virtual bool initialize(std::vector<cv::VideoCapture>& cameras) override;
    virtual bool isInitialized() const override;
    virtual std::vector<DartDetection> detectDarts(const std::vector<cv::Mat>& frames) override;

private:
    bool initialized;
    bool debug_mode;
    int target_width, target_height, target_fps;
    // Your custom detector state here
};
```

### 2. Implement the Factory Function

```cpp
// myCoolDetector.cpp
#include "myCoolDetector.hpp"

// REQUIRED: Export factory function with C linkage
extern "C" DetectorInterface* create_detector(bool debug_mode, int width, int height, int fps) {
    return new MyCoolDetector(debug_mode, width, height, fps);
}

MyCoolDetector::MyCoolDetector(bool debug_mode, int width, int height, int fps)
    : initialized(false), debug_mode(debug_mode),
      target_width(width), target_height(height), target_fps(fps) {
    // Initialize your detector
}

bool MyCoolDetector::initialize(std::vector<cv::VideoCapture>& cameras) {
    // Your calibration/initialization logic here
    // Return true if successful
    initialized = true;
    return true;
}

std::vector<DartDetection> MyCoolDetector::detectDarts(const std::vector<cv::Mat>& frames) {
    std::vector<DartDetection> detections;

    // Your dart detection logic here
    // Process frames and return detections

    return detections;
}
```

### 3. Build as Shared Library

#### Simple Plugin (single file):

```bash
g++ -shared -fPIC -I/path/to/opendartboard/src \
  myCoolDetector.cpp \
  -lopencv_core -lopencv_imgproc \
  -o detectors/myCoolDetector.so
```

#### Complex Plugin (multiple files):

```bash
# Build all components into one shared library
g++ -shared -fPIC -I/path/to/opendartboard/src \
  myCoolDetector.cpp \
  advanced_algo.cpp \
  helper_utils.cpp \
  -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
  -ltensorflow \
  -o detectors/myCoolDetector/libmyCoolDetector.so
```

## Plugin Directory Structure

### Simple Plugin Layout:

```
detectors/
├── libmyCoolDetector.so     # Single shared library
├── libAwesomeDetector.so    # Another simple plugin
└── libSuperDetector.so      # Yet another plugin
```

### Advanced Plugin Layout:

```
detectors/
├── MyCoolDetector/
│   ├── libMyCoolDetector.so  # Main plugin library
│   ├── models/
│   │   ├── dartboard.pb      # TensorFlow model
│   │   └── calibration.yaml  # Configuration
│   ├── lib/
│   │   ├── opencv_utils.so   # Helper libraries
│   │   └── ai_engine.so      # AI inference engine
│   ├── config.json           # Plugin configuration
│   └── README.md             # Plugin documentation
├── AwesomeDetector/
│   ├── libAwesomeDetector.so
│   ├── yolo_weights.bin
│   └── config.json
└── SuperDetector/
    ├── libSuperDetector.so
    ├── custom_model.onnx
    └── README.md
```

## Example Plugins

### 1. YOLO-based Detector

```cpp
// YOLODetector.cpp
#include "detector_interface.hpp"
#include <opencv2/dnn.hpp>

class YOLODetector : public DetectorInterface {
private:
    cv::dnn::Net net;
    bool initialized;

public:
    YOLODetector(bool debug, int w, int h, int fps) : initialized(false) {
        // Load YOLO model
        net = cv::dnn::readNetFromDarknet("detectors/YOLO/yolo.cfg",
                                          "detectors/YOLO/yolo.weights");
    }

    bool initialize(std::vector<cv::VideoCapture>& cameras) override {
        initialized = !net.empty();
        return initialized;
    }

    std::vector<DartDetection> detectDarts(const std::vector<cv::Mat>& frames) override {
        // YOLO detection logic
        std::vector<DartDetection> detections;
        for (const auto& frame : frames) {
            // Run YOLO inference
            // Convert to DartDetection format
        }
        return detections;
    }
};

extern "C" DetectorInterface* create_detector(bool debug, int w, int h, int fps) {
    return new YOLODetector(debug, w, h, fps);
}
```

### 2. OpenCV Template Matching Detector

```cpp
// TemplateDetector.cpp
class TemplateDetector : public DetectorInterface {
private:
    cv::Mat dartTemplate;

public:
    TemplateDetector(bool debug, int w, int h, int fps) {
        // Load dart template image
        dartTemplate = cv::imread("detectors/Template/dart_template.jpg", 0);
    }

    std::vector<DartDetection> detectDarts(const std::vector<cv::Mat>& frames) override {
        std::vector<DartDetection> detections;

        for (size_t i = 0; i < frames.size(); i++) {
            cv::Mat gray;
            cv::cvtColor(frames[i], gray, cv::COLOR_BGR2GRAY);

            cv::Mat result;
            cv::matchTemplate(gray, dartTemplate, result, cv::TM_CCOEFF_NORMED);

            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

            if (maxVal > 0.8) {  // Threshold
                DartDetection detection;
                detection.position = maxLoc;
                detection.confidence = maxVal;
                detection.camera_index = i;
                detection.score = "TEMPLATE_HIT";
                detections.push_back(detection);
            }
        }

        return detections;
    }
};
```

## Data Structures

### DartDetection

```cpp
struct DartDetection {
    cv::Point position;       // Dart position in image
    float confidence;         // Detection confidence (0.0 - 1.0)
    std::string score;        // Calculated score (e.g., "20", "BULL", "MISS")
    int camera_index;         // Which camera detected this
};
```

### DetectorInterface

```cpp
class DetectorInterface {
public:
    virtual ~DetectorInterface() = default;
    virtual bool initialize(std::vector<cv::VideoCapture>& cameras) = 0;
    virtual bool isInitialized() const = 0;
    virtual std::vector<DartDetection> detectDarts(const std::vector<cv::Mat>& frames) = 0;
};
```

## Installation & Testing

### Install Plugin

```bash
# Copy plugin to detectors directory
mkdir -p detectors/MyCoolDetector
cp libMyCoolDetector.so detectors/MyCoolDetector/

# Test the plugin
opendartboard --detector=MyCoolDetector --debug
```

### Debug Plugin Loading

```bash
# Enable debug mode to see plugin loading messages
opendartboard --detector=MyCoolDetector --debug

# Check logs for:
# "Attempting to load custom detector: MyCoolDetector"
# "Successfully loaded custom detector: MyCoolDetector"
```

## Error Handling

If plugin loading fails, OpenDartboard will:

1. Try multiple file locations
2. Log detailed error messages
3. Fall back to geometry detector
4. Continue running normally

Common issues:

- Missing `create_detector` function
- Incorrect library dependencies
- Permission issues
- Missing include paths

## Advanced Features

### Plugin Configuration

```json
// detectors/MyCoolDetector/config.json
{
  "name": "MyCoolDetector",
  "version": "1.0.0",
  "author": "Your Name",
  "description": "Advanced dart detection using custom algorithms",
  "dependencies": ["opencv >= 4.0", "tensorflow >= 2.0"],
  "parameters": {
    "confidence_threshold": 0.8,
    "nms_threshold": 0.4
  }
}
```

### Multi-library Plugins

```cpp
// In your main plugin, load additional libraries
void* helper_lib = dlopen("detectors/MyCoolDetector/helper.so", RTLD_LAZY);
typedef void (*init_helper_t)();
init_helper_t init_helper = (init_helper_t) dlsym(helper_lib, "init_helper");
init_helper();
```

## Future Plugin Repository

We plan to create a plugin repository where developers can:

- Share detector plugins
- Download community detectors
- Rate and review plugins
- Automatic dependency management

Stay tuned for updates! 🚀
