#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp> // <--- ADDED BACK for SIFT
#include "camera.hpp"
#include "logger.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>

// --- A more flexible Settings structure ---
struct AppSettings {
    // General
    std::string feature_detector = "GFTT";
    int camera_downscale_factor = 2;
    double ransac_threshold = 1.0;
    int max_trail_length = 3;

    // GFTT Params
    struct GFTT {
        int max_features = 1000;
        double quality_level = 0.01;
        double min_distance = 10.0;
        double redetection_ratio = 0.9;
    } gftt;

    // SIFT Params
    struct SIFT {
        int max_features = 500;
        double redetection_ratio = 0.8;
    } sift;

    // ORB Params
    struct ORB {
        int max_features = 1000;
        float scale_factor = 1.2f;
        int n_levels = 8;
        int edge_threshold = 31;
        double redetection_ratio = 0.9;
    } orb;
};

// --- Updated loading function ---
void loadSettings(AppSettings& settings, const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        std::cout << "Successfully loaded config file: " << filename << std::endl;

        settings.feature_detector = config["feature_detector"].as<std::string>(settings.feature_detector);
        settings.camera_downscale_factor = config["camera_downscale_factor"].as<int>(settings.camera_downscale_factor);
        settings.ransac_threshold = config["ransac_threshold"].as<double>(settings.ransac_threshold);
        settings.max_trail_length = config["max_trail_length"].as<int>(settings.max_trail_length);

        // Load GFTT params
        if (config["gftt"]) {
            settings.gftt.max_features = config["gftt"]["max_features"].as<int>(settings.gftt.max_features);
            settings.gftt.quality_level = config["gftt"]["quality_level"].as<double>(settings.gftt.quality_level);
            settings.gftt.min_distance = config["gftt"]["min_distance"].as<double>(settings.gftt.min_distance);
            settings.gftt.redetection_ratio = config["gftt"]["redetection_ratio"].as<double>(settings.gftt.redetection_ratio);
        }
        // Load SIFT params
        if (config["sift"]) {
            settings.sift.max_features = config["sift"]["max_features"].as<int>(settings.sift.max_features);
            settings.sift.redetection_ratio = config["sift"]["redetection_ratio"].as<double>(settings.sift.redetection_ratio);
        }
        // Load ORB params
        if (config["orb"]) {
            settings.orb.max_features = config["orb"]["max_features"].as<int>(settings.orb.max_features);
            settings.orb.scale_factor = config["orb"]["scale_factor"].as<float>(settings.orb.scale_factor);
            settings.orb.n_levels = config["orb"]["n_levels"].as<int>(settings.orb.n_levels);
            settings.orb.edge_threshold = config["orb"]["edge_threshold"].as<int>(settings.orb.edge_threshold);
            settings.orb.redetection_ratio = config["orb"]["redetection_ratio"].as<double>(settings.orb.redetection_ratio);
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "WARN: Could not load or parse config file. Using default settings. Error: " << e.what() << std::endl;
    }
}

// --- ADDED: Unified feature detection function ---
void detectFeatures(cv::Mat image, std::vector<cv::Point2f>& points, const AppSettings& settings, cv::Ptr<cv::Feature2D>& detector) {
    points.clear();
    if (settings.feature_detector == "GFTT") {
        cv::goodFeaturesToTrack(image, points, settings.gftt.max_features, settings.gftt.quality_level, settings.gftt.min_distance);
    } else {
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(image, keypoints);
        cv::KeyPoint::convert(keypoints, points);
    }
}

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
    AppSettings settings;
    loadSettings(settings, "/home/doer/Desktop/gps_denied_ws/vio_project/config/config.yaml");

    std::cout << "Starting VIO Project with detector: " << settings.feature_detector << std::endl;

    // --- ADDED: Create the selected detector object ---
    cv::Ptr<cv::Feature2D> detector;
    if (settings.feature_detector == "SIFT") {
        detector = cv::SIFT::create(settings.sift.max_features);
    } else if (settings.feature_detector == "ORB") {
        detector = cv::ORB::create(settings.orb.max_features, settings.orb.scale_factor, settings.orb.n_levels, settings.orb.edge_threshold);
    }
    // GFTT does not use a detector object, it's handled in detectFeatures.

    PerformanceLogger logger("performance_log.csv");
    Camera cam(0);
    if (!cam.open()) return -1;
    cam.printCameraInfo();

    cv::Mat frame, current_gray_frame, gray_frame_for_sift, prev_gray_frame_for_sift;
    std::map<int, std::deque<cv::Point2f>> track_history;
    int next_track_id = 0;

    const int sift_process_width = cam.getWidth() / settings.camera_downscale_factor;
    const int sift_process_height = cam.getHeight() / settings.camera_downscale_factor;
    const cv::Mat K = (cv::Mat_<double>(3, 3) << sift_process_width, 0, sift_process_width / 2.0, 0, sift_process_width, sift_process_height / 2.0, 0, 0, 1);
    cv::Mat R_global = cv::Mat::eye(3, 3, CV_64F), t_global = cv::Mat::zeros(3, 1, CV_64F);
    bool first_frame = true;

    while (true) {
        if (!cam.readFrame(frame)) break;

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
                    t_global = t; R_global = R;
                }
                std::map<int, std::deque<cv::Point2f>> updated_track_history;
                for (size_t i = 0; i < status.size(); ++i) {
                    if (status[i]) {
                        int id = track_ids[i];
                        auto history = track_history[id];
                        history.push_back(current_points[i]);
                        while (history.size() > (size_t)settings.max_trail_length) history.pop_front();
                        updated_track_history[id] = history;
                    }
                }
                track_history = updated_track_history;
            }

            // --- MODIFIED: Redetection logic uses the unified function ---
            double redetection_ratio = (settings.feature_detector == "GFTT") ? settings.gftt.redetection_ratio :
                                       (settings.feature_detector == "SIFT") ? settings.sift.redetection_ratio :
                                       settings.orb.redetection_ratio;
            int max_features = (settings.feature_detector == "GFTT") ? settings.gftt.max_features :
                               (settings.feature_detector == "SIFT") ? settings.sift.max_features :
                               settings.orb.max_features;

            if (track_history.size() < (size_t)(max_features * redetection_ratio)) {
                std::vector<cv::Point2f> new_points;
                detectFeatures(gray_frame_for_sift, new_points, settings, detector);
                for (const auto& point : new_points) {
                    bool is_new = true;
                    for (const auto& pair : track_history) {
                        if (cv::norm(point - pair.second.back()) < settings.gftt.min_distance) {
                            is_new = false; break;
                        }
                    }
                    if (is_new) track_history[next_track_id++].push_back(point);
                }
            }
        } else {
            // --- MODIFIED: Initial detection uses the unified function ---
            std::vector<cv::Point2f> initial_points;
            detectFeatures(gray_frame_for_sift, initial_points, settings, detector);
            for (const auto& point : initial_points) {
                track_history[next_track_id++].push_back(point);
            }
            first_frame = false;
        }

        double roll, pitch, yaw;
        rotationMatrixToRPY(R_global, roll, pitch, yaw);
        std::cout << std::fixed << std::setprecision(2) << "\r" << "Translation: [" << t_global.at<double>(0) << ", " << t_global.at<double>(1) << ", " << t_global.at<double>(2) << "]  RPY: [" << roll << ", " << pitch << ", " << yaw << "] rad";
        std::cout.flush();
        
        cv::Mat frame_with_tracks;
        cv::resize(frame, frame_with_tracks, cv::Size(sift_process_width, sift_process_height));
        for (const auto& pair : track_history) {
            const auto& trail = pair.second;
            for (size_t i = 1; i < trail.size(); ++i) cv::line(frame_with_tracks, trail[i - 1], trail[i], cv::Scalar(0, 255, 0), 1);
            if (!trail.empty()) cv::circle(frame_with_tracks, trail.back(), 3, cv::Scalar(255, 0, 0), -1);
        }
        
        cv::imshow("Feature Trails", frame_with_tracks);
        if (cv::waitKey(1) == 'q') break;
        prev_gray_frame_for_sift = gray_frame_for_sift.clone();
    }

    std::cout << std::endl << "VIO Project Finished." << std::endl;
    cv::destroyAllWindows();
    return 0;
}