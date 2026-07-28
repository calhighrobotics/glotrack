#pragma once

#include "PoseLogger.hpp"
#include "KalmanFilter.hpp"
#include <atomic>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

struct CameraFocusSettings {
    bool autoFocus = false;
    int focusValue = 0;
    int maxFocus = 255;
};

class AppContext {
public:
    AppContext();

    PoseLogger poseLogger;
    KalmanFilter kalmanFilter;
    RobotPose2D currentPose;

    std::atomic<bool> camera_running{true};
    std::mutex frame_mutex;
    cv::Mat shared_frame;

    cv::VideoCapture cap;
    int cameraIndex = 0;

    cv::Mat K;
    cv::Mat D;
    bool isCalibrated = false;
    bool showCalibrationWindow = false;
    bool isTrackingActive = false;
};

inline AppContext::AppContext() {
    K = cv::Mat::eye(3, 3, CV_64F);
    D = cv::Mat::zeros(5, 1, CV_64F);
}