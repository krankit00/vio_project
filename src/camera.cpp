#include "camera.hpp"
#include <iostream>

/**
 * @brief Constructor for the Camera class.
 * @param camera_index The index of the camera to open (e.g., 0 for default webcam).
 */
Camera::Camera(int camera_index) : camera_index_(camera_index) {
    // Constructor initializes the camera index.
    // The actual camera opening is done in the open() method.
}

/**
 * @brief Destructor for the Camera class. Releases the camera.
 */
Camera::~Camera() {
    close(); // Ensure the camera is released when the object is destroyed
}

/**
 * @brief Opens the camera with optional initial parameters.
 * @param width Desired frame width. Use -1 for default.
 * @param height Desired frame height. Use -1 for default.
 * @param fps Desired frames per second. Use -1.0 for default.
 * @param exposure Desired exposure value. Use -1.0 for auto exposure.
 * @return True if the camera was opened successfully, false otherwise.
 */
bool Camera::open(int width, int height, double fps, double exposure) {
    // Set properties BEFORE opening the camera for better compatibility with some backends (like GStreamer)
    // Note: Not all cameras support setting properties before opening.
    // OpenCV will attempt to set them, but actual values might differ.
    if (width != -1 && height != -1) {
        setResolution(width, height);
    }
    if (fps != -1.0) {
        setFPS(fps);
    }
    if (exposure != -1.0) {
        setExposure(exposure);
    }

    cap_.open(camera_index_);
    if (!cap_.isOpened()) {
        std::cerr << "ERROR: Could not open camera with index " << camera_index_ 
                  << ". Check if camera is connected and not in use." << std::endl;
        return false;
    }
    std::cout << "Camera with index " << camera_index_ << " opened successfully." << std::endl;
    return true;
}

/**
 * @brief Closes the camera.
 */
void Camera::close() {
    if (cap_.isOpened()) {
        cap_.release();
        std::cout << "Camera released." << std::endl;
    }
}

/**
 * @brief Checks if the camera is currently open.
 * @return True if the camera is open, false otherwise.
 */
bool Camera::isOpened() const {
    return cap_.isOpened();
}

/**
 * @brief Reads a new frame from the camera.
 * @param frame Output parameter to store the captured frame.
 * @return True if a frame was successfully read, false otherwise.
 */
bool Camera::readFrame(cv::Mat& frame) {
    if (!cap_.isOpened()) {
        std::cerr << "ERROR: Cannot read frame, camera is not open." << std::endl;
        return false;
    }
    cap_ >> frame;
    if (frame.empty()) {
        std::cerr << "ERROR: Blank frame grabbed, or camera disconnected." << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief Sets the desired resolution for the camera.
 * @param width The desired frame width.
 * @param height The desired frame height.
 * @return True if the resolution was set (or attempted to be set) successfully, false otherwise.
 * Note: Cameras may not support all requested resolutions, and will pick the closest.
 */
bool Camera::setResolution(int width, int height) {
    // Set properties on the VideoCapture object.
    // This can be called before or after cap_.open()
    bool success_width = cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    bool success_height = cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    if (!success_width || !success_height) {
        std::cerr << "WARNING: Failed to set requested resolution to " << width << "x" << height << "." << std::endl;
        return false;
    }
    // std::cout << "Attempted to set resolution to: " << width << "x" << height << std::endl; // Logged in printCameraInfo
    return true;
}

/**
 * @brief Gets the current frame width.
 * @return The current frame width, or -1.0 if camera is not open.
 */
double Camera::getWidth() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_FRAME_WIDTH) : -1.0;
}

/**
 * @brief Gets the current frame height.
 * @return The current frame height, or -1.0 if camera is not open.
 */
double Camera::getHeight() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_FRAME_HEIGHT) : -1.0;
}

/**
 * @brief Sets the desired frames per second (FPS) for the camera.
 * @param fps The desired FPS.
 * @return True if the FPS was set (or attempted to be set) successfully, false otherwise.
 * Note: Cameras may not support all requested FPS values.
 */
bool Camera::setFPS(double fps) {
    bool success = cap_.set(cv::CAP_PROP_FPS, fps);
    if (!success) {
        std::cerr << "WARNING: Failed to set requested FPS to " << fps << "." << std::endl;
    }
    // else { std::cout << "Attempted to set FPS to: " << fps << std::endl; } // Logged in printCameraInfo
    return success;
}

/**
 * @brief Gets the current frames per second (FPS) of the camera.
 * @return The current FPS, or -1.0 if camera is not open.
 */
double Camera::getFPS() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_FPS) : -1.0;
}

/**
 * @brief Sets the exposure value for the camera.
 * @param exposure The desired exposure value.
 * @return True if the exposure was set successfully, false otherwise.
 * Note: Exposure values are camera-specific. Use -1.0 for auto exposure.
 */
bool Camera::setExposure(double exposure) {
    bool success = cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
    if (!success) {
        std::cerr << "WARNING: Failed to set exposure to " << exposure << ". Not all cameras support this." << std::endl;
    }
    // else { std::cout << "Attempted to set exposure to: " << exposure << std::endl; } // Logged in printCameraInfo
    return success;
}

/**
 * @brief Gets the current exposure value of the camera.
 * @return The current exposure value, or -1.0 if camera is not open.
 */
double Camera::getExposure() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_EXPOSURE) : -1.0;
}

/**
 * @brief Sets the brightness value for the camera.
 * @param brightness The desired brightness value.
 * @return True if the brightness was set successfully, false otherwise.
 */
bool Camera::setBrightness(double brightness) {
    bool success = cap_.set(cv::CAP_PROP_BRIGHTNESS, brightness);
    if (!success) {
        std::cerr << "WARNING: Failed to set brightness to " << brightness << ". Not all cameras support this." << std::endl;
    }
    return success;
}

/**
 * @brief Gets the current brightness value of the camera.
 * @return The current brightness value, or -1.0 if camera is not open.
 */
double Camera::getBrightness() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_BRIGHTNESS) : -1.0;
}

/**
 * @brief Sets the contrast value for the camera.
 * @param contrast The desired contrast value.
 * @return True if the contrast was set successfully, false otherwise.
 */
bool Camera::setContrast(double contrast) {
    bool success = cap_.set(cv::CAP_PROP_CONTRAST, contrast);
    if (!success) {
        std::cerr << "WARNING: Failed to set contrast to " << contrast << ". Not all cameras support this." << std::endl;
    }
    return success;
}

/**
 * @brief Gets the current contrast value of the camera.
 * @return The current contrast value, or -1.0 if camera is not open.
 */
double Camera::getContrast() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_CONTRAST) : -1.0;
}

/**
 * @brief Sets the saturation value for the camera.
 * @param saturation The desired saturation value.
 * @return True if the saturation was set successfully, false otherwise.
 */
bool Camera::setSaturation(double saturation) {
    bool success = cap_.set(cv::CAP_PROP_SATURATION, saturation);
    if (!success) {
        std::cerr << "WARNING: Failed to set saturation to " << saturation << ". Not all cameras support this." << std::endl;
    }
    return success;
}

/**
 * @brief Gets the current saturation value of the camera.
 * @return The current saturation value, or -1.0 if camera is not open.
 */
double Camera::getSaturation() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_SATURATION) : -1.0;
}

/**
 * @brief Sets the hue value for the camera.
 * @param hue The desired hue value.
 * @return True if the hue was set successfully, false otherwise.
 */
bool Camera::setHue(double hue) {
    bool success = cap_.set(cv::CAP_PROP_HUE, hue);
    if (!success) {
        std::cerr << "WARNING: Failed to set hue to " << hue << ". Not all cameras support this." << std::endl;
    }
    return success;
}

/**
 * @brief Gets the current hue value of the camera.
 * @return The current hue value, or -1.0 if camera is not open.
 */
double Camera::getHue() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_HUE) : -1.0;
}

/**
 * @brief Sets the gain value for the camera.
 * @param gain The desired gain value.
 * @return True if the gain was set successfully, false otherwise.
 */
bool Camera::setGain(double gain) {
    bool success = cap_.set(cv::CAP_PROP_GAIN, gain);
    if (!success) {
        std::cerr << "WARNING: Failed to set gain to " << gain << ". Not all cameras support this." << std::endl;
    }
    return success;
}

/**
 * @brief Gets the current gain value of the camera.
 * @return The current gain value, or -1.0 if camera is not open.
 */
double Camera::getGain() const {
    return cap_.isOpened() ? cap_.get(cv::CAP_PROP_GAIN) : -1.0;
}

/**
 * @brief Prints all current camera parameters to the console.
 */
void Camera::printCameraInfo() const {
    if (!cap_.isOpened()) {
        std::cout << "Camera is not open. Cannot print parameters." << std::endl;
        return;
    }
    std::cout << "--- Current Camera Parameters ---" << std::endl;
    std::cout << "Actual Resolution: " << getWidth() << "x" << getHeight() << std::endl;
    std::cout << "Actual FPS: " << getFPS() << std::endl;
    std::cout << "Exposure: " << getExposure() << " (Note: -1.0 usually means auto)" << std::endl;
    std::cout << "Brightness: " << getBrightness() << std::endl;
    std::cout << "Contrast: " << getContrast() << std::endl;
    std::cout << "Saturation: " << getSaturation() << std::endl;
    std::cout << "Hue: " << getHue() << std::endl;
    std::cout << "Gain: " << getGain() << std::endl;
    std::cout << "---------------------------------" << std::endl;
}
