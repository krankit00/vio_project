#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief A class to encapsulate camera operations and properties.
 * It provides methods to open/close the camera, read frames,
 * and get/set various camera parameters like resolution, FPS, and exposure.
 */
class Camera {
public:
    /**
     * @brief Constructor for the Camera class.
     * @param camera_index The index of the camera to open (e.g., 0 for default webcam).
     */
    Camera(int camera_index = 0);

    /**
     * @brief Destructor for the Camera class. Releases the camera.
     */
    ~Camera();

    /**
     * @brief Opens the camera with optional initial parameters.
     * @param width Desired frame width. Use -1 for default.
     * @param height Desired frame height. Use -1 for default.
     * @param fps Desired frames per second. Use -1.0 for default.
     * @param exposure Desired exposure value. Use -1.0 for auto exposure.
     * @return True if the camera was opened successfully, false otherwise.
     */
    bool open(int width = -1, int height = -1, double fps = -1.0, double exposure = -1.0);

    /**
     * @brief Closes the camera.
     */
    void close();

    /**
     * @brief Checks if the camera is currently open.
     * @return True if the camera is open, false otherwise.
     */
    bool isOpened() const;

    /**
     * @brief Reads a new frame from the camera.
     * @param frame Output parameter to store the captured frame.
     * @return True if a frame was successfully read, false otherwise.
     */
    bool readFrame(cv::Mat& frame);

    /**
     * @brief Sets the desired resolution for the camera.
     * @param width The desired frame width.
     * @param height The desired frame height.
     * @return True if the resolution was set (or attempted to be set) successfully, false otherwise.
     * Note: Cameras may not support all requested resolutions, and will pick the closest.
     */
    bool setResolution(int width, int height);

    /**
     * @brief Gets the current frame width.
     * @return The current frame width, or -1.0 if camera is not open.
     */
    double getWidth() const;

    /**
     * @brief Gets the current frame height.
     * @return The current frame height, or -1.0 if camera is not open.
     */
    double getHeight() const;

    /**
     * @brief Sets the desired frames per second (FPS) for the camera.
     * @param fps The desired FPS.
     * @return True if the FPS was set (or attempted to be set) successfully, false otherwise.
     * Note: Cameras may not support all requested FPS values.
     */
    bool setFPS(double fps);

    /**
     * @brief Gets the current frames per second (FPS) of the camera.
     * @return The current FPS, or -1.0 if camera is not open.
     */
    double getFPS() const;

    /**
     * @brief Sets the exposure value for the camera.
     * @param exposure The desired exposure value.
     * @return True if the exposure was set successfully, false otherwise.
     * Note: Exposure values are camera-specific. Use -1.0 for auto exposure.
     */
    bool setExposure(double exposure);

    /**
     * @brief Gets the current exposure value of the camera.
     * @return The current exposure value, or -1.0 if camera is not open.
     */
    double getExposure() const;

    /**
     * @brief Sets the brightness value for the camera.
     * @param brightness The desired brightness value.
     * @return True if the brightness was set successfully, false otherwise.
     */
    bool setBrightness(double brightness);

    /**
     * @brief Gets the current brightness value of the camera.
     * @return The current brightness value, or -1.0 if camera is not open.
     */
    double getBrightness() const;

    /**
     * @brief Sets the contrast value for the camera.
     * @param contrast The desired contrast value.
     * @return True if the contrast was set successfully, false otherwise.
     */
    bool setContrast(double contrast);

    /**
     * @brief Gets the current contrast value of the camera.
     * @return The current contrast value, or -1.0 if camera is not open.
     */
    double getContrast() const;

    /**
     * @brief Sets the saturation value for the camera.
     * @param saturation The desired saturation value.
     * @return True if the saturation was set successfully, false otherwise.
     */
    bool setSaturation(double saturation);

    /**
     * @brief Gets the current saturation value of the camera.
     * @return The current saturation value, or -1.0 if camera is not open.
     */
    double getSaturation() const;

    /**
     * @brief Sets the hue value for the camera.
     * @param hue The desired hue value.
     * @return True if the hue was set successfully, false otherwise.
     */
    bool setHue(double hue);

    /**
     * @brief Gets the current hue value of the camera.
     * @return The current hue value, or -1.0 if camera is not open.
     */
    double getHue() const;

    /**
     * @brief Sets the gain value for the camera.
     * @param gain The desired gain value.
     * @return True if the gain was set successfully, false otherwise.
     */
    bool setGain(double gain);

    /**
     * @brief Gets the current gain value of the camera.
     * @return The current gain value, or -1.0 if camera is not open.
     */
    double getGain() const;

    /**
     * @brief Prints all current camera parameters to the console.
     */
    void printCameraInfo() const;

private:
    cv::VideoCapture cap_; ///< OpenCV VideoCapture object.
    int camera_index_;     ///< Index of the camera being used.
};

#endif // CAMERA_HPP
