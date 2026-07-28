// Copyright 2026 DataTopping
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <GLES3/gl3.h>
#include <GLFW/glfw3.h>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

bool saveCameraCalibration(const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

bool loadCameraCalibration(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

void glfw_error_callback(int error, const char* description);

void matToTexture(cv::Mat& frame, GLuint& textureID);

GLuint loadTextureFromFile(const std::string& filepath);

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                 const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                 const std::vector<cv::Mat>& rvecs,
                                 const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs,
                                 std::vector<float>& perViewErrors);