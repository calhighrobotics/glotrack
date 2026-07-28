// Copyright 2026 DataTopping
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "AppContext.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <map>
#include <vector>

class LocalizationPipeline {
public:
    LocalizationPipeline();

    void renderLocalizationWindow(AppContext& ctx, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

private:
    bool computeLocalizationPose(const cv::Mat& frame,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs,
                                 const std::map<int, std::vector<cv::Point3f>>& worldFieldMap,
                                 RobotPose2D& outPose,
                                 std::map<int, bool>& outTagVisibility,
                                 AppContext& ctx);

    cv::Mat cached_R_cam_T;
    cv::Mat cached_tvec_cam;
    bool isCameraCalibrated;

    cv::aruco::Dictionary dictionary;
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector;

    bool isFieldOriented;
    bool isCameraLocked;
    bool resetCalibrationRequested;
    
    double robotTagSize;
    double fieldTagSize;
    double robotTagHeightZ;
    double robotTagOffsetX;
    double robotTagOffsetY;
    
    float tag1Center[3];
    float tag2Center[3];
    float tag3Center[3];
    float tag4Center[3];
    
    bool isLoggingActive;
    LogTarget currentLogTarget;
    PoseLogger logger;
};