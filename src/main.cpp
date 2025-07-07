#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "camera.hpp"
#include "config.hpp" // New
#include "logger.hpp" // New

#include <map>
#include <deque>
#include <chrono>

int main() {
    // --- 1. Load Configuration ---
    Config config;
    config.load("../config/config.yaml"); // Assumes build dir is inside project root

    // --- 2. Initialize Logger ---
    PerformanceLogger logger("performance_log.csv");
    auto last_log_time = std::chrono::steady_clock::now();

    // --- 3. Setup Camera and SIFT ---
    std::cout << "Starting VIO Project - RANSAC Tracking Demo" << std::endl;
    Camera cam(config.cameraIndex());
    if (!cam.open(config.resolution().width, config.resolution().height, config.fps(), -1.0)) {
        return -1;
    }
    cam.printCameraInfo();

    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(config.maxSiftFeatures());

    // --- 4. Initialize Tracking Variables ---
    cv::Mat frame, current_gray_frame, gray_frame_for_sift, prev_gray_frame_for_sift;
    std::map<int, std::deque<cv::Point2f>> track_history;
    int next_track_id = 0;
    bool first_frame = true;

    // --- 5. Main Loop ---
    while (true) {
        if (!cam.readFrame(frame)) break;

        // Grayscale conversion and resizing
        cv::cvtColor(frame, current_gray_frame, cv::COLOR_BGR2GRAY);
        cv::resize(current_gray_frame, gray_frame_for_sift, config.siftProcessResolution());

        if (!first_frame) {
            std::vector<cv::Point2f> prev_points, current_points;
            std::vector<int> track_ids;
            for (const auto& pair : track_history) {
                prev_points.push_back(pair.second.back());
                track_ids.push_back(pair.first);
            }

            if (prev_points.size() > 8) { // Need at least 8 points for RANSAC
                // --- Optical Flow ---
                std::vector<uchar> status;
                cv::calcOpticalFlowPyrLK(prev_gray_frame_for_sift, gray_frame_for_sift, prev_points, current_points, status, cv::noArray());

                // --- RANSAC Outlier Rejection ---
                if (config.useRansac()) {
                    cv::findFundamentalMat(prev_points, current_points, cv::FM_RANSAC, config.ransacThreshold(), 0.99, status);
                }

                // --- Update Track History with Inliers ---
                std::map<int, std::deque<cv::Point2f>> updated_history;
                for (size_t i = 0; i < status.size(); ++i) {
                    if (status[i]) { // Only keep inliers
                        int id = track_ids[i];
                        auto history = track_history[id];
                        history.push_back(current_points[i]);
                        while (history.size() > config.maxTrailLength()) {
                            history.pop_front();
                        }
                        updated_history[id] = history;
                    }
                }
                track_history = updated_history;
            } else {
                track_history.clear(); // Not enough points, reset
            }
        }

        // --- Redetection Logic ---
        const int redetection_threshold = config.maxSiftFeatures() * config.redetectionThresholdRatio();
        if (track_history.size() < redetection_threshold) {
            std::vector<cv::KeyPoint> new_keypoints;
            sift->detect(gray_frame_for_sift, new_keypoints);
            for (const auto& kp : new_keypoints) {
                bool is_new = true;
                for (const auto& pair : track_history) {
                    if (cv::norm(kp.pt - pair.second.back()) < config.minFeatureDistance()) {
                        is_new = false;
                        break;
                    }
                }
                if (is_new) {
                    track_history[next_track_id++].push_back(kp.pt);
                }
            }
            std::cout << "Feature count low. Redetected. Total features: " << track_history.size() << std::endl;
        }

        first_frame = false;
        prev_gray_frame_for_sift = gray_frame_for_sift.clone();

        // --- Visualization ---
        cv::Mat frame_with_tracks;
        cv::resize(frame, frame_with_tracks, config.siftProcessResolution());
        for (const auto& pair : track_history) {
            const auto& trail = pair.second;
            for (size_t i = 1; i < trail.size(); ++i) {
                cv::line(frame_with_tracks, trail[i - 1], trail[i], cv::Scalar(0, 255, 0), 1);
            }
            if (!trail.empty()) {
                cv::circle(frame_with_tracks, trail.back(), 3, cv::Scalar(255, 0, 0), -1);
            }
        }
        cv::imshow("RANSAC Feature Tracking", frame_with_tracks);

        // --- Logging ---
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
            logger.log();
            last_log_time = now;
        }
        
        if (cv::waitKey(1) == 'q') break;
    }

    cv::destroyAllWindows();
    std::cout << "VIO Project Finished." << std::endl;
    return 0;
}