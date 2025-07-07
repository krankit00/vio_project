#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "camera.hpp"

// For tracking history
#include <map>
#include <deque>
#include "logger.hpp"
#include <chrono>

int main() {
    std::cout << "Starting VIO Project - Feature Tracking with Trails" << std::endl;

    PerformanceLogger logger("performance_log.csv");
    auto last_log_time = std::chrono::steady_clock::now();

    Camera cam(0);
    if (!cam.open()) {
        return -1;
    }
    cam.printCameraInfo();

    const int max_sift_features = 500;
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(max_sift_features);

    cv::Mat frame, current_gray_frame, gray_frame_for_sift, prev_gray_frame_for_sift, frame_with_tracks;
    std::map<int, std::deque<cv::Point2f>> track_history;
    int next_track_id = 0;

    // --- Configuration for Tracking and Visualization ---
    const int max_trail_length = 3;
    const int min_feature_distance = 5;
    const int redetection_threshold = max_sift_features / 1;
    const double RANSAC_THRESHOLD = 1.0;

    const int max_display_width = 1280;
    const int sift_process_width = 640;
    const int sift_process_height = 480;

    // --- ADDED: Camera Intrinsics (Pinhole Model) ---
    // NOTE: These are assumed values. For accurate results, you should calibrate your camera.
    const double fx = sift_process_width; // Assume focal length is image width
    const double fy = sift_process_width;
    const double cx = sift_process_width / 2.0;
    const double cy = sift_process_height / 2.0;
    const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // --- ADDED: Global Pose Variables ---
    cv::Mat R_global = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_global = cv::Mat::zeros(3, 1, CV_64F);

    double prev_time = static_cast<double>(cv::getTickCount());
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

                // --- MODIFIED: Use Essential Matrix for Pose Estimation ---
                // Replaces findFundamentalMat for better accuracy with known camera intrinsics.
                cv::Mat E, R, t;
                E = cv::findEssentialMat(prev_points, current_points, K, cv::RANSAC, 0.999, RANSAC_THRESHOLD, status);

                if (!E.empty()) {
                    cv::recoverPose(E, prev_points, current_points, K, R, t, status);

                    // --- ADDED: Accumulate global pose ---
                    // Note: Translation is up to scale.
                    t_global = t_global + R_global * t;
                    R_global = R * R_global;
                }
                
                std::map<int, std::deque<cv::Point2f>> updated_track_history;
                for (size_t i = 0; i < status.size(); ++i) {
                    if (status[i]) {
                        int id = track_ids[i];
                        auto history = track_history[id];
                        history.push_back(current_points[i]);
                        while (history.size() > max_trail_length) {
                            history.pop_front();
                        }
                        updated_track_history[id] = history;
                    }
                }
                track_history = updated_track_history;
            }

            if (track_history.size() < redetection_threshold) {
                std::vector<cv::KeyPoint> new_keypoints;
                sift->detect(gray_frame_for_sift, new_keypoints);
                int added_count = 0;
                for (const auto& kp : new_keypoints) {
                    bool is_new = true;
                    for (const auto& pair : track_history) {
                        double dist = cv::norm(kp.pt - pair.second.back());
                        if (dist < min_feature_distance) {
                            is_new = false;
                            break;
                        }
                    }
                    if (is_new) {
                        track_history[next_track_id++].push_back(kp.pt);
                        added_count++;
                    }
                }
            }
        } else {
            std::vector<cv::KeyPoint> initial_keypoints;
            sift->detect(gray_frame_for_sift, initial_keypoints);
            for (const auto& kp : initial_keypoints) {
                track_history[next_track_id++].push_back(kp.pt);
            }
            first_frame = false;
        }

        // --- ADDED: Print Pose Information to Terminal ---
        // Using .t() to transpose the 3x1 vector to a 1x3 for cleaner printing.
        std::cout << "Translation: " << t_global.t() << " Rotation (first row): " << R_global.row(0) << std::endl;
        
        // Drawing and display logic remains the same...
        cv::Mat color_for_drawing;
        cv::resize(frame, color_for_drawing, cv::Size(sift_process_width, sift_process_height));
        frame_with_tracks = color_for_drawing.clone();
        for (const auto& pair : track_history) {
            const auto& trail = pair.second;
            for (size_t i = 1; i < trail.size(); ++i) {
                cv::line(frame_with_tracks, trail[i - 1], trail[i], cv::Scalar(0, 255, 0), 1);
            }
            if (!trail.empty()) {
                cv::circle(frame_with_tracks, trail.back(), 3, cv::Scalar(255, 0, 0), -1);
            }
        }
        prev_gray_frame_for_sift = gray_frame_for_sift.clone();
        double current_time = static_cast<double>(cv::getTickCount());
        double fps = cv::getTickFrequency() / (current_time - prev_time);
        prev_time = current_time;
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
        cv::putText(frame_with_tracks, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
        
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
            logger.log();
            last_log_time = now;
        }
        
        cv::Mat resized_display_frame;
        cv::resize(frame_with_tracks, resized_display_frame, cv::Size(max_display_width, static_cast<int>(max_display_width * ((double)frame_with_tracks.rows / frame_with_tracks.cols))));
        cv::imshow("Feature Trails", resized_display_frame);

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cv::destroyAllWindows();
    std::cout << "VIO Project Finished." << std::endl;
    return 0;
}