# C++ Monocular Visual Odometry

A real-time monocular visual odometry project developed in C++. This application tracks features between consecutive camera frames to estimate the camera's motion (rotation and translation). It's built to be flexible, allowing for easy configuration and experimentation with different feature detectors and parameters via a YAML file.

> *(This is a sample GIF. You can create your own using a screen recording tool like Peek or ScreenToGif.)*

## 🚀 Features

- **Real-time Pose Estimation**: Calculates the camera's 6-DoF motion (3D rotation and translation) frame-by-frame.
- **Switchable Feature Detectors**: Easily switch between GFTT, SIFT, and ORB feature detectors by changing a single line in the configuration file.
- **Robust Tracking**: Uses KLT optical flow to track features and RANSAC to reject outliers when estimating motion.
- **External Configuration**: All parameters are managed in a `config.yaml` file, allowing for tuning without recompiling the code.
- **Performance Logging**: Includes a simple logger to monitor CPU and RAM usage, outputting to `performance_log.csv`.

## 🛠 Dependencies

To build and run this project, you'll need the following:

- C++17 compatible compiler (like GCC or Clang)
- CMake (version 3.10 or higher)
- OpenCV (version 4.x recommended), including the `xfeatures2d` module for SIFT.
- `yaml-cpp` library for parsing the configuration file.

On a Debian-based system (like Ubuntu), install all dependencies with:
```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev libopencv-xfeatures2d-dev libyaml-cpp-dev
```

## 🏗️ How to Build and Run

### 1. Clone the Repository
```bash
git clone [<your-repository-url>](https://github.com/krankit00/vio_project.git)
cd vio_project
```

### 2. Build the Project

We use CMake for a simple, out-of-source build.
```bash
# Create a build directory
mkdir build && cd build

# Generate the build files
cmake ..

# Compile the project
make
```

The executable `vio_executable` will be created in the `build/src` directory.

### 3. Run the Application

Important: Always run the executable from within the `build` directory so it can correctly find the config folder.
```bash
# From within the 'build/src' directory
./vio_executable
```

Press `q` in the display window to quit the application.

## ⚙️ Configuration

All parameters can be modified in the `config/config.yaml` file without needing to recompile the code.

### Switching Feature Detectors

To change the feature detector, modify the `feature_detector` key in the YAML file. Options: `"GFTT"`, `"SIFT"`, and `"ORB"`.

```yaml
# Select the feature detector: "GFTT", "SIFT", or "ORB"
feature_detector: "GFTT"

# --- Camera and Pose Estimation ---
camera_downscale_factor: 2
ransac_threshold: 1.0 # RANSAC outlier threshold in pixels

# ===================================================================
# --- Detector-Specific Parameters ---
# ===================================================================

gftt:
  max_features: 1000
  quality_level: 0.01
  min_distance: 10.0
  redetection_ratio: 0.9

sift:
  max_features: 500
  redetection_ratio: 0.8

orb:
  max_features: 1000
  scale_factor: 1.2
  n_levels: 8
  redetection_ratio: 0.9
```

## 🔮 Project Roadmap & Future Improvements

This project provides a solid foundation for a visual odometry system. Some key areas for future improvement:

- 📷 **Camera Calibration**: Calibrate your camera to reduce error by obtaining accurate intrinsic parameters and distortion coefficients.
- 📏 **Scale Estimation**: Resolve scale ambiguity by integrating an IMU (Visual-Inertial Odometry) or stereo vision.
- 🔗 **Backend Optimization**: Use graph optimization (e.g., Bundle Adjustment) to correct for accumulated drift.
- 🔄 **Loop Closing**: Recognize previously seen locations to minimize long-term drift.

---

Made with ❤️ for robotics and computer vision research.