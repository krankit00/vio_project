#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <opencv2/opencv.hpp>

class Config {
public:
    // Tries to load the config file, returns false on failure
    bool load(const std::string& filename);

    // Getters for all parameters
    int cameraIndex() const { return camera_index_; }
    cv::Size resolution() const { return resolution_; }
    int fps() const { return fps_; }
    cv::Size siftProcessResolution() const { return sift_process_resolution_; }
    int maxSiftFeatures() const { return max_sift_features_; }
    double redetectionThresholdRatio() const { return redetection_threshold_ratio_; }
    int minFeatureDistance() const { return min_feature_distance_; }
    bool useRansac() const { return use_ransac_; }
    double ransacThreshold() const { return ransac_threshold_; }
    int maxTrailLength() const { return max_trail_length_; }

private:
    // Camera
    int camera_index_ = 0;
    cv::Size resolution_ = {1280, 720};
    int fps_ = 30;

    // Processing
    cv::Size sift_process_resolution_ = {640, 480};

    // Features
    int max_sift_features_ = 500;
    double redetection_threshold_ratio_ = 0.5;
    int min_feature_distance_ = 10;

    // RANSAC
    bool use_ransac_ = true;
    double ransac_threshold_ = 1.0;

    // Visualization
    int max_trail_length_ = 15;
};

#endif // CONFIG_HPP