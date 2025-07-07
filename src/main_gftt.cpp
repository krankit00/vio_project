#include <iostream>
#include <iomanip> // for std::setprecision
#include <opencv2/opencv.hpp>
#include "camera.hpp"

// For tracking history
#include <map>
#include <deque>
#include "logger.hpp"
#include <chrono>
#include <opencv2/core.hpp>
#include <cmath>
#include <yaml-cpp/yaml.h> // <--- ADDED: YAML header

// --- ADDED: A struct to hold all configurable parameters ---
struct AppSettings {
    // Features
    int max_features = 1000;
    double quality_level = 0.01;
    double min_feature_distance = 10.0;
    double redetection_ratio = 0.9;

    // Pose Estimation
    double ransac_threshold = 1.0;

    // Visualization
    int max_trail_length = 3;

    // Camera
    int downscale_factor = 2;
};

// --- ADDED: Function to load settings from the YAML file ---
void loadSettings(AppSettings& settings, const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        std::cout << "Successfully loaded config file: " << filename << std::endl;

        settings.max_features = config["max_features"].as<int>(settings.max_features);
        settings.quality_level = config["quality_level"].as<double>(settings.quality_level);
        settings.min_feature_distance = config["min_feature_distance"].as<double>(settings.min_feature_distance);
        settings.redetection_ratio = config["redetection_ratio"].as<double>(settings.redetection_ratio);
        settings.ransac_threshold = config["ransac_threshold"].as<double>(settings.ransac_threshold);
        settings.max_trail_length = config["max_trail_length"].as<int>(settings.max_trail_length);
        settings.downscale_factor = config["downscale_factor"].as<int>(settings.downscale_factor);

    } catch (const YAML::Exception& e) {
        std::cerr << "WARN: Could not load or parse config file. Using default settings. Error: " << e.what() << std::endl;
    }
}

// Converts a 3x3 rotation matrix to roll, pitch, yaw (in radians)
void rotationMatrixToRPY(const cv::Mat& R, double& roll, double& pitch, double& yaw) {
    pitch = std::asin(-R.at<double>(2,0));
    if (std::cos(pitch) > 1e-6) {
        roll = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
        yaw  = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        roll = 0;
        yaw = std::atan2(-R.at<double>(0,1), R.at<double>(1,1));
    }
}

int main() {
    // --- MODIFIED: Load settings from file at startup ---
    AppSettings settings;
    loadSettings(settings, "/home/doer/Desktop/gps_denied_ws/vio_project/config/config.yaml"); // Assumes build dir is inside project root

    std::cout << "Starting VIO Project - Feature Tracking with Trails" << std::endl;

    PerformanceLogger logger("performance_log.csv");
    auto last_log_time = std::chrono::steady_clock::now();

    Camera cam(0);
    if (!cam.open()) {
        return -1;
    }
    cam.printCameraInfo();

    cv::Mat frame, current_gray_frame, gray_frame_for_sift, prev_gray_frame_for_sift;
    std::map<int, std::deque<cv::Point2f>> track_history;
    int next_track_id = 0;

    const int max_display_width = 1280;
    const int sift_process_width = cam.getWidth() / settings.downscale_factor;
    const int sift_process_height = cam.getHeight() / settings.downscale_factor;

    const double fx = sift_process_width;
    const double fy = sift_process_width;
    const double cx = sift_process_width / 2.0;
    const double cy = sift_process_height / 2.0;
    const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    cv::Mat R_global = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_global = cv::Mat::zeros(3, 1, CV_64F);

    bool first_frame = true;

    while (true) {
        if (!cam.readFrame(frame)) {
            break;
        }

        cv::cvtColor(frame, current_gray_frame, cv::COLOR_BGR2GRAY);
        cv::resize(current_gray_frame, gray_frame_for_sift, cv::Size(sift_process_width, sift_process_height));

        if (!first_frame) {
            std::vector<cv::Point2f> prev_points, current_points;
            std::vector<int> track_ids;

            for (const auto& pair : track_history) {
                prev_points.push_back(pair.second.back());
                track_ids.push_back(pair.first);
            }

            if (prev_points.size() > 8) {
                std::vector<uchar> status;
                cv::calcOpticalFlowPyrLK(prev_gray_frame_for_sift, gray_frame_for_sift, prev_points, current_points, status, cv::noArray());

                cv::Mat E, R, t;
                E = cv::findEssentialMat(prev_points, current_points, K, cv::RANSAC, 0.999, settings.ransac_threshold, status);

                if (!E.empty()) {
                    cv::recoverPose(E, prev_points, current_points, K, R, t, status);
                    t_global = t;
                    R_global = R;
                }
                
                std::map<int, std::deque<cv::Point2f>> updated_track_history;
                for (size_t i = 0; i < status.size(); ++i) {
                    if (status[i]) {
                        int id = track_ids[i];
                        auto history = track_history[id];
                        history.push_back(current_points[i]);
                        while (history.size() > (size_t)settings.max_trail_length) {
                            history.pop_front();
                        }
                        updated_track_history[id] = history;
                    }
                }
                track_history = updated_track_history;
            }

            const int redetection_threshold = settings.max_features * settings.redetection_ratio;
            if (track_history.size() < (size_t)redetection_threshold) {
                std::vector<cv::Point2f> new_corners;
                // --- MODIFIED: Use settings for GFTT parameters ---
                cv::goodFeaturesToTrack(gray_frame_for_sift, new_corners, settings.max_features, settings.quality_level, settings.min_feature_distance);
                
                for (const auto& corner : new_corners) {
                    bool is_new = true;
                    for (const auto& pair : track_history) {
                        if (cv::norm(corner - pair.second.back()) < settings.min_feature_distance) {
                            is_new = false;
                            break;
                        }
                    }
                    if (is_new) {
                        track_history[next_track_id++].push_back(corner);
                    }
                }
            }
        } else {
            std::vector<cv::Point2f> initial_corners;
             // --- MODIFIED: Use settings for GFTT parameters ---
            cv::goodFeaturesToTrack(gray_frame_for_sift, initial_corners, settings.max_features, settings.quality_level, settings.min_feature_distance);
            
            for (const auto& corner : initial_corners) {
                track_history[next_track_id++].push_back(corner);
            }
            first_frame = false;
        }

        double roll, pitch, yaw;
        rotationMatrixToRPY(R_global, roll, pitch, yaw);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "\r" << "Translation: [" 
              << t_global.at<double>(0) << ", "
              << t_global.at<double>(1) << ", "
              << t_global.at<double>(2) << "]  "
              << "RPY: [" << roll << ", " << pitch << ", " << yaw << "] rad";
        std::cout.flush();
        
        cv::Mat frame_with_tracks;
        cv::resize(frame, frame_with_tracks, cv::Size(sift_process_width, sift_process_height));
        for (const auto& pair : track_history) {
            const auto& trail = pair.second;
            for (size_t i = 1; i < trail.size(); ++i) {
                cv::line(frame_with_tracks, trail[i - 1], trail[i], cv::Scalar(0, 255, 0), 1);
            }
            if (!trail.empty()) {
                cv::circle(frame_with_tracks, trail.back(), 3, cv::Scalar(255, 0, 0), -1);
            }
        }
        
        cv::Mat resized_display_frame;
        cv::resize(frame_with_tracks, resized_display_frame, cv::Size(max_display_width, static_cast<int>(max_display_width * ((double)frame_with_tracks.rows / frame_with_tracks.cols))));
        cv::imshow("Feature Trails", resized_display_frame);

        if (cv::waitKey(1) == 'q') {
            break;
        }
        prev_gray_frame_for_sift = gray_frame_for_sift.clone();
    }

    std::cout << std::endl << "VIO Project Finished." << std::endl;
    cv::destroyAllWindows();
    return 0;
}