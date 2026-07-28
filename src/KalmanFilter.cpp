// Copyright 2026 DataTopping
// SPDX-License-Identifier: GPL-3.0-or-later

#include "KalmanFilter.hpp"

#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>

KalmanFilter::KalmanFilter(float processNoise, float measurementNoise) 
    : kf(4, 2, 0), isInitialized(false) 
{
    kf.transitionMatrix = cv::Mat::eye(4, 4, CV_32F);
    kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(processNoise));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measurementNoise));
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1.0));
}

cv::Point2f KalmanFilter::predict() {
    auto currentTime = std::chrono::high_resolution_clock::now();
    
    if (!isInitialized) {
        lastTime = currentTime;
        return cv::Point2f(0.0f, 0.0f);
    }
    float dt = std::chrono::duration<float>(currentTime - lastTime).count();
    lastTime = currentTime;
    dt = std::clamp(dt, 0.001f, 0.2f);
    kf.transitionMatrix.at<float>(0, 2) = dt;
    kf.transitionMatrix.at<float>(1, 3) = dt;
    cv::Mat statePrediction = kf.predict();
    return cv::Point2f(statePrediction.at<float>(0), statePrediction.at<float>(1));
}

cv::Point2f KalmanFilter::update(float measuredX, float measuredY) {
    if (!isInitialized) {
        kf.statePost.at<float>(0) = measuredX;
        kf.statePost.at<float>(1) = measuredY;
        kf.statePost.at<float>(2) = 0.0f;
        kf.statePost.at<float>(3) = 0.0f;
        
        lastTime = std::chrono::high_resolution_clock::now();
        isInitialized = true;
        return cv::Point2f(measuredX, measuredY);
    }

    cv::Mat measurement = (cv::Mat_<float>(2, 1) << measuredX, measuredY);
    cv::Mat stateCorrected = kf.correct(measurement);
    return cv::Point2f(stateCorrected.at<float>(0), stateCorrected.at<float>(1));
}

void KalmanFilter::reset() { isInitialized = false; }
bool KalmanFilter::initialized() const { return isInitialized; }