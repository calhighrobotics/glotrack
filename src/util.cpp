// Copyright 2026 DataTopping
// SPDX-License-Identifier: GPL-3.0-or-later

#include "util.hpp"
#include <iostream>
#include <opencv2/calib.hpp>
#include <opencv2/geometry/3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

bool saveCameraCalibration(const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    if (cameraMatrix.empty() || distCoeffs.empty()) {
        std::cerr << "[Error] Camera matrix or distortion coefficients are empty!\n";
        return false;
    }

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "[Error] Failed to open file for writing: " << filename << "\n";
        return false;
    }

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();

    std::cout << "[Success] Saved camera calibration to: " << filename << "\n";
    return true;
}

bool loadCameraCalibration(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[Error] Failed to open calibration file: " << filename << "\n";
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    if (cameraMatrix.empty() || distCoeffs.empty()) {
        std::cerr << "[Error] File read failed or missing calibration keys!\n";
        return false;
    }

    std::cout << "[Success] Loaded camera calibration from: " << filename << "\n";
    return true;
}

void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void matToTexture(cv::Mat& frame, GLuint& textureID) {
    if (frame.empty())
        return;
    cv::Mat rgbFrame;
    cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgbFrame.cols, rgbFrame.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, rgbFrame.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

GLuint loadTextureFromFile(const std::string& filepath) {
    cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "[Error] Could not load background image: " << filepath << "\n";
        return 0;
    }
    cv::cvtColor(img, img, cv::COLOR_BGR2RGBA);
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.cols, img.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    return textureID;
}

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                 const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                 const std::vector<cv::Mat>& rvecs,
                                 const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs,
                                 std::vector<float>& perViewErrors) {
    std::vector<cv::Point2f> projectedPoints;
    size_t totalPoints = 0;
    double totalErr = 0.0;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projectedPoints);
        double err = cv::norm(imagePoints[i], projectedPoints, cv::NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = static_cast<float>(std::sqrt((err * err) / n));
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}