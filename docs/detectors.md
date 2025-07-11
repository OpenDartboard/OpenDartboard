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

### 1. Define the Interface

Your detector must implement this interface:

```cpp
// detector_interface.hpp (copy this to your project)
struct DetectorResult {
    bool dart_detected = false;
    std::string score = "";
    cv::Point2f position{-1, -1};  // Dart position for overlay
    float confidence = 0.0f;
    int camera_index = -1;
    uint64_t timestamp = 0;

    // Metadata
    bool motion_detected = false;
    int processing_time_ms = 0;

    // Easy boolean check
    operator bool() const { return dart_detected; }
};

class DetectorInterface {
public:
    virtual ~DetectorInterface() = default;
    virtual bool initialize(std::vector<cv::VideoCapture>& cameras) = 0;
    virtual bool isInitialized() const = 0;
    virtual DetectorResult process(const std::vector<cv::Mat>& frames) = 0;
};
```

### 2. Implement Your Detector

```cpp
// myCoolDetector.hpp
#include "detector_interface.hpp"

class MyCoolDetector : public DetectorInterface {
public:
    MyCoolDetector(bool debug_mode, int width, int height, int fps);
    virtual ~MyCoolDetector() = default;

    virtual bool initialize(std::vector<cv::VideoCapture>& cameras) override;
    virtual bool isInitialized() const override;
    virtual DetectorResult process(const std::vector<cv::Mat>& frames) override;

private:
    bool initialized = false;
    bool debug_mode;
    int target_width, target_height, target_fps;
    // Your custom detector state here
};
```

### 3. Implement Your Logic

```cpp
// myCoolDetector.cpp
#include "myCoolDetector.hpp"

// REQUIRED: Export factory function with C linkage
extern "C" DetectorInterface* create_detector(bool debug_mode, int width, int height, int fps) {
    return new MyCoolDetector(debug_mode, width, height, fps);
}

MyCoolDetector::MyCoolDetector(bool debug_mode, int width, int height, int fps)
    : debug_mode(debug_mode), target_width(width), target_height(height), target_fps(fps) {
}

bool MyCoolDetector::initialize(std::vector<cv::VideoCapture>& cameras) {
    // Your calibration/initialization logic here
    // Return true if successful
    initialized = true;
    return true;
}

bool MyCoolDetector::isInitialized() const {
    return initialized;
}

DetectorResult MyCoolDetector::process(const std::vector<cv::Mat>& frames) {
    auto start_time = std::chrono::steady_clock::now();
    DetectorResult result;
    result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    if (!initialized || frames.empty()) {
        return result;
    }

    // YOUR DETECTION LOGIC HERE
    // Analyze frames and detect dart

    // Example: Simple template matching
    if (/* dart detected */) {
        result.dart_detected = true;
        result.score = "S20";  // Your calculated score
        result.position = cv::Point2f(320, 240);  // Dart position
        result.confidence = 0.95f;  // Detection confidence
        result.camera_index = 0;  // Which camera detected it
    }

    // Calculate processing time
    auto end_time = std::chrono::steady_clock::now();
    result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();

    return result;
}
```

### 4. Build as Shared Library

```bash
# Simple build
g++ -shared -fPIC \
  myCoolDetector.cpp \
  -lopencv_core -lopencv_imgproc \
  -o detectors/myCoolDetector.so

# Or with folder structure
mkdir -p detectors/MyCoolDetector
g++ -shared -fPIC \
  myCoolDetector.cpp \
  advanced_algo.cpp \
  -lopencv_core -lopencv_imgproc \
  -ltensorflow \
  -o detectors/MyCoolDetector/libMyCoolDetector.so
```

## Plugin Directory Structure

### Simple Layout:

```
detectors/
├── libmyCoolDetector.so
├── libAwesomeDetector.so
└── libSuperDetector.so
```

### Advanced Layout:

```
detectors/
├── MyCoolDetector/
│   ├── libMyCoolDetector.so
│   ├── models/
│   │   └── dartboard_model.pb
│   ├── config.json
│   └── README.md
└── AwesomeDetector/
    ├── libAwesomeDetector.so
    └── yolo_weights.bin
```

## Example Detectors

### YOLO-based Detector

```cpp
#include "detector_interface.hpp"
#include <opencv2/dnn.hpp>

class YOLODetector : public DetectorInterface {
private:
    cv::dnn::Net net;
    bool initialized = false;

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

    bool isInitialized() const override {
        return initialized;
    }

    DetectorResult process(const std::vector<cv::Mat>& frames) override {
        DetectorResult result;

        for (size_t i = 0; i < frames.size(); i++) {
            // YOLO inference
            cv::Mat blob;
            cv::dnn::blobFromImage(frames[i], blob, 1/255.0, cv::Size(416, 416));
            net.setInput(blob);

            std::vector<cv::Mat> outputs;
            net.forward(outputs, net.getUnconnectedOutLayersNames());

            // Process YOLO output
            if (/* dart detected */) {
                result.dart_detected = true;
                result.score = "T20";
                result.position = cv::Point2f(/* x, y */);
                result.confidence = /* confidence */;
                result.camera_index = i;
                break;
            }
        }

        return result;
    }
};

extern "C" DetectorInterface* create_detector(bool debug, int w, int h, int fps) {
    return new YOLODetector(debug, w, h, fps);
}
```

### Template Matching Detector

```cpp
class TemplateDetector : public DetectorInterface {
private:
    cv::Mat dartTemplate;
    bool initialized = false;

public:
    TemplateDetector(bool debug, int w, int h, int fps) {
        // Load dart template image
        dartTemplate = cv::imread("detectors/Template/dart_template.jpg", 0);
    }

    DetectorResult process(const std::vector<cv::Mat>& frames) override {
        DetectorResult result;

        for (size_t i = 0; i < frames.size(); i++) {
            cv::Mat gray;
            cv::cvtColor(frames[i], gray, cv::COLOR_BGR2GRAY);

            cv::Mat matchResult;
            cv::matchTemplate(gray, dartTemplate, matchResult, cv::TM_CCOEFF_NORMED);

            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc);

            if (maxVal > 0.8) {
                result.dart_detected = true;
                result.position = cv::Point2f(maxLoc.x, maxLoc.y);
                result.confidence = maxVal;
                result.camera_index = i;
                result.score = "S16";
                break;
            }
        }

        return result;
    }
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

## How It Works

1. **OpenDartboard calls your detector**: `DetectorResult result = detector->process(frames)`
2. **Your detector analyzes frames**: Use any computer vision techniques you want
3. **Return rich result data**: Position, score, confidence, etc.
4. **OpenDartboard handles the rest**: WebSocket communication, game logic, etc.

Your detector doesn't need to know about:

- WebSocket protocols
- Game state management
- Camera initialization
- Frame rate control

Just focus on: **frames in → dart detection out**

## Error Handling

If plugin loading fails, OpenDartboard will:

1. Try multiple file locations
2. Log detailed error messages
3. Fall back to geometry detector

Common issues:

- Missing `create_detector` function
- Incorrect library dependencies
- Permission issues

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
    "confidence_threshold": 0.8
  }
}
```

### Multi-library Plugins

```cpp
// Load additional libraries in your detector
void* helper_lib = dlopen("detectors/MyCoolDetector/helper.so", RTLD_LAZY);
```

Your detector has complete freedom to:

- Use any AI frameworks (TensorFlow, PyTorch, ONNX)
- Load additional shared libraries
- Use any computer vision techniques
- Implement custom calibration logic
- Handle multiple cameras however you want

## Future Plugin Repository

We plan to create a plugin repository where developers can:

- Share detector plugins
- Download community detectors
- Rate and review plugins

Stay tuned for updates!
