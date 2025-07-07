#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "camera.hpp"

// For tracking history
#include <map>
#include <deque>
#include "logger.hpp" // <--- ADDED: Logger header
#include <chrono>   // <--- ADDED: For timing logger calls

int main() {
    std::cout << "Starting VIO Project - Feature Tracking with Trails" << std::endl;

    // --- ADDED: Initialize Logger ---
    PerformanceLogger logger("performance_log.csv");
    auto last_log_time = std::chrono::steady_clock::now();

    // Create a Camera object (using default camera index 0)
    Camera cam(0);

    // Open the camera, attempting to set desired resolution, FPS, and exposure
    if (!cam.open()) { // width, height, fps, exposure
        return -1; // Exit if camera cannot be opened
    }

    cam.printCameraInfo();

    const int max_sift_features = 500;
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(max_sift_features);

    cv::Mat frame;              // Original frame from camera
    cv::Mat current_gray_frame; // Grayscale frame for SIFT and optical flow
    cv::Mat gray_frame_for_sift; // Resized grayscale frame for processing
    cv::Mat prev_gray_frame_for_sift; // Previous grayscale frame for optical flow
    cv::Mat frame_with_tracks; // Output frame for display

    std::map<int, std::deque<cv::Point2f>> track_history;
    int next_track_id = 0; 

    // --- Configuration for Tracking and Visualization ---
    const int max_trail_length = 3; 
    const int min_feature_distance = 5; 
    const int redetection_threshold = max_sift_features / 10; 
    const double RANSAC_THRESHOLD = 1.0; // <--- ADDED: RANSAC pixel threshold

    const int max_display_width = 1280;
    const int sift_process_width = 640;
    const int sift_process_height = 480;

    double prev_time = static_cast<double>(cv::getTickCount());
    bool first_frame = true;

    while (true) {
        if (!cam.readFrame(frame)) {
            break;
        }

        if (frame.channels() == 3 || frame.channels() == 4) {
            cv::cvtColor(frame, current_gray_frame, cv::COLOR_BGR2GRAY);
        } else {
            current_gray_frame = frame.clone();

        }

        cv::resize(current_gray_frame, gray_frame_for_sift, cv::Size(sift_process_width, sift_process_height));

        // --- Feature Tracking and Redetection Logic ---
        if (!first_frame) {
            std::vector<cv::Point2f> prev_points, current_points;
            std::vector<int> track_ids;

            for (const auto& pair : track_history) {
                prev_points.push_back(pair.second.back());
                track_ids.push_back(pair.first);
            }

            if (prev_points.size() > 8) { // Minimum points for RANSAC
                std::vector<uchar> status;
                std::vector<float> err;
                cv::calcOpticalFlowPyrLK(prev_gray_frame_for_sift, gray_frame_for_sift,
                                         prev_points, current_points,
                                         status, err, cv::Size(21, 21), 3);

                // --- ADDED: RANSAC Outlier Rejection ---
                // This re-uses the status vector. Points that fail optical flow are already
                // marked as 0. RANSAC will then mark its outliers as 0 as well.
                cv::findFundamentalMat(prev_points, current_points, cv::FM_RANSAC, RANSAC_THRESHOLD, 0.99, status);


                // Update track history with new points and remove lost ones
                std::map<int, std::deque<cv::Point2f>> updated_track_history;
                for (size_t i = 0; i < status.size(); ++i) {
                    if (status[i]) { // Status now contains only optical flow successes AND RANSAC inliers
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
                std::cout << "Tracked features (after RANSAC): " << track_history.size() << std::endl;
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
                        track_history[next_track_id].push_back(kp.pt);
                        next_track_id++;
                        added_count++;
                    }
                }
                std::cout << "Redetected and added " << added_count << " new features." << std::endl;
            }

        } else {
            std::vector<cv::KeyPoint> initial_keypoints;
            sift->detect(gray_frame_for_sift, initial_keypoints);
            for (const auto& kp : initial_keypoints) {
                track_history[next_track_id].push_back(kp.pt);
                next_track_id++;
            }
            first_frame = false;
            std::cout << "Initial SIFT features detected: " << track_history.size() << std::endl;
        }

        // --- Drawing ---
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
        cv::putText(frame_with_tracks, fps_text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

        // --- ADDED: Periodic Logging ---
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
            logger.log();
            last_log_time = now;
        }

        double display_aspect_ratio = (double)frame_with_tracks.cols / frame_with_tracks.rows;
        int new_display_width = max_display_width;
        int new_display_height = static_cast<int>(new_display_width / display_aspect_ratio);

        cv::Mat resized_display_frame;
        cv::resize(frame_with_tracks, resized_display_frame, cv::Size(new_display_width, new_display_height));

        cv::imshow("Feature Trails", resized_display_frame);

        if (cv::waitKey(1) == 'q') {
            std::cout << "Quitting." << std::endl;
            break;
        }
    }

    cv::destroyAllWindows();
    std::cout << "VIO Project Finished." << std::endl;
    return 0;
}