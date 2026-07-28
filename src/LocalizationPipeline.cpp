// Copyright 2026 DataTopping
// SPDX-License-Identifier: GPL-3.0-or-later

#include "LocalizationPipeline.hpp"
#include "imgui.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <string>

LocalizationPipeline::LocalizationPipeline()
    : isCameraCalibrated(false),
      isFieldOriented(false),
      isCameraLocked(false),
      resetCalibrationRequested(false),
      robotTagSize(6.5),
      fieldTagSize(6.5),
      robotTagHeightZ(12.0),
      robotTagOffsetX(0.0),
      robotTagOffsetY(0.0),
      tag1Center{-52.8125f, 5.0f, 0.0f},
      tag2Center{67.0f, 59.375f, 0.0f},
      tag3Center{67.0f, -67.0f, 0.0f},
      tag4Center{-52.8125f, -67.0f, 0.0f},
      isLoggingActive(false),
      currentLogTarget(LogTarget::Console) {
    
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    detectorParams.adaptiveThreshWinSizeMax = 45;
    detectorParams.adaptiveThreshWinSizeStep = 5;
    detectorParams.polygonalApproxAccuracyRate = 0.06;
    detectorParams.minCornerDistanceRate = 0.02;
    detectorParams.cornerRefinementMaxIterations = 20;
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
}

bool LocalizationPipeline::computeLocalizationPose(const cv::Mat& frame,
                                                   const cv::Mat& cameraMatrix,
                                                   const cv::Mat& distCoeffs,
                                                   const std::map<int, std::vector<cv::Point3f>>& worldFieldMap,
                                                   RobotPose2D& outPose,
                                                   std::map<int, bool>& outTagVisibility,
                                                   AppContext& ctx) {
    isFieldOriented = false;
    outTagVisibility = {
        {0, false}, {1, false}, {2, false}, {3, false}, {4, false}
    };

    if (frame.empty() || cameraMatrix.empty() || distCoeffs.empty()) {
        return false;
    }

    cv::Point2f predictedPos = ctx.kalmanFilter.predict();

    if (resetCalibrationRequested) {
        isCameraCalibrated = false;
        cached_R_cam_T.release();
        cached_tvec_cam.release();
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    detector.detectMarkers(frame, corners, ids, rejected);

    std::map<int, size_t> idToIndexMap;
    for (size_t i = 0; i < ids.size(); ++i) {
        int currentId = ids[i];
        if (outTagVisibility.find(currentId) != outTagVisibility.end()) {
            outTagVisibility[currentId] = true;
            idToIndexMap[currentId] = i;
        }
    }

    if (!isCameraLocked && !isCameraCalibrated) {
        std::vector<cv::Point3f> accumulatedWorldPoints;
        std::vector<cv::Point2f> accumulatedPixelPoints;

        for (int boundaryId = 1; boundaryId <= 4; ++boundaryId) {
            if (outTagVisibility[boundaryId]) {
                size_t idx = idToIndexMap[boundaryId];
                const auto& worldCoords = worldFieldMap.at(boundaryId);

                for (int c = 0; c < 4; ++c) {
                    accumulatedWorldPoints.push_back(worldCoords[c]);
                    accumulatedPixelPoints.push_back(corners[idx][c]);
                }
            }
        }

        if (accumulatedWorldPoints.size() >= 4) {
            cv::Mat rvec_cam;
            bool pnpSuccess = cv::solvePnP(accumulatedWorldPoints, accumulatedPixelPoints, cameraMatrix, distCoeffs,
                                           rvec_cam, cached_tvec_cam, false, cv::SOLVEPNP_ITERATIVE);
            if (pnpSuccess) {
                cv::Mat R_cam;
                cv::Rodrigues(rvec_cam, R_cam);
                cached_R_cam_T = R_cam.t();
                isCameraCalibrated = true;
            }
        }
    }

    isFieldOriented = isCameraCalibrated || isCameraLocked;

    if (!isFieldOriented) {
        if (ctx.kalmanFilter.initialized()) {
            outPose.x = predictedPos.x;
            outPose.y = predictedPos.y;
        }
        return false;
    }

    if (outTagVisibility[0]) {
        const double halfRobotTag = robotTagSize / 2.0;

        const std::vector<cv::Point3f> localTagCoords = {
            cv::Point3f(static_cast<float>(-halfRobotTag + robotTagOffsetX), static_cast<float>( halfRobotTag + robotTagOffsetY), static_cast<float>(robotTagHeightZ)),
            cv::Point3f(static_cast<float>( halfRobotTag + robotTagOffsetX), static_cast<float>( halfRobotTag + robotTagOffsetY), static_cast<float>(robotTagHeightZ)),
            cv::Point3f(static_cast<float>( halfRobotTag + robotTagOffsetX), static_cast<float>(-halfRobotTag + robotTagOffsetY), static_cast<float>(robotTagHeightZ)),
            cv::Point3f(static_cast<float>(-halfRobotTag + robotTagOffsetX), static_cast<float>(-halfRobotTag + robotTagOffsetY), static_cast<float>(robotTagHeightZ))
        };

        size_t robotIdx = idToIndexMap[0];
        cv::Mat rvec_robot, tvec_robot;

        bool robotPnpSuccess = cv::solvePnP(localTagCoords, corners[robotIdx], cameraMatrix, distCoeffs, rvec_robot,
                                            tvec_robot, false, cv::SOLVEPNP_ITERATIVE);

        if (robotPnpSuccess) {
            cv::Mat R_R_in_C;
            cv::Rodrigues(rvec_robot, R_R_in_C);

            cv::Mat robot_world_position = cached_R_cam_T * (tvec_robot - cached_tvec_cam);
            cv::Mat robot_world_rotation = cached_R_cam_T * R_R_in_C;

            double rawX = robot_world_position.at<double>(0);
            double rawY = robot_world_position.at<double>(1);

            cv::Point2f correctedPos = ctx.kalmanFilter.update(static_cast<float>(rawX), static_cast<float>(rawY));
            outPose.x = correctedPos.x;
            outPose.y = correctedPos.y;

            double fwd_x = robot_world_rotation.at<double>(0, 1);
            double fwd_y = -robot_world_rotation.at<double>(1, 1);

            double headingDeg = std::atan2(-fwd_x, fwd_y) * (180.0 / CV_PI);
            if (headingDeg < 0.0) {
                headingDeg += 360.0;
            }

            outPose.theta = headingDeg;
            return true;
        }
    }

    if (ctx.kalmanFilter.initialized()) {
        outPose.x = predictedPos.x;
        outPose.y = predictedPos.y;
    }

    return false;
}

void LocalizationPipeline::renderLocalizationWindow(AppContext& ctx,
                                                    const cv::Mat& cameraMatrix,
                                                    const cv::Mat& distCoeffs) {
    ImGui::BeginChild("Localization", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Physical Configurations (Inches):");
    if (ctx.isTrackingActive) {
        ImGui::BeginDisabled();
    }
    ImGui::InputDouble("Robot Tag Size", &robotTagSize, 0.1, 1.0, "%.2f");
    ImGui::InputDouble("Field Tag Size", &fieldTagSize, 0.1, 1.0, "%.2f");
    ImGui::Spacing();
    ImGui::TextUnformatted("Robot Tag Mounting Offsets (Relative to Floor Center):");
    ImGui::InputDouble("Tag Height (Z Ground Offset)", &robotTagHeightZ, 0.1, 1.0, "%.2f");
    ImGui::InputDouble("Tag Offset X (Right+/Left-)", &robotTagOffsetX, 0.1, 1.0, "%.2f");
    ImGui::InputDouble("Tag Offset Y (Fwd+/Bwd-)", &robotTagOffsetY, 0.1, 1.0, "%.2f");
    ImGui::Spacing();
    ImGui::TextUnformatted("Field AprilTag Center Coordinates (X, Y, Z):");
    ImGui::InputFloat3("Tag 1 (Top-Left)", tag1Center, "%.1f");
    ImGui::InputFloat3("Tag 2 (Top-Right)", tag2Center, "%.1f");
    ImGui::InputFloat3("Tag 3 (Bottom-Right)", tag3Center, "%.1f");
    ImGui::InputFloat3("Tag 4 (Bottom-Left)", tag4Center, "%.1f");
    if (ctx.isTrackingActive) {
        ImGui::EndDisabled();
    }
    ImGui::Separator();
    const float halfFieldTag = static_cast<float>(fieldTagSize / 2.0);
    std::map<int, std::vector<cv::Point3f>> worldFieldMap = {
        {1,
         {cv::Point3f(tag1Center[0] - halfFieldTag, tag1Center[1] + halfFieldTag, tag1Center[2]),
          cv::Point3f(tag1Center[0] + halfFieldTag, tag1Center[1] + halfFieldTag, tag1Center[2]),
          cv::Point3f(tag1Center[0] + halfFieldTag, tag1Center[1] - halfFieldTag, tag1Center[2]),
          cv::Point3f(tag1Center[0] - halfFieldTag, tag1Center[1] - halfFieldTag, tag1Center[2])}},
        {2,
         {cv::Point3f(tag2Center[0] - halfFieldTag, tag2Center[1] + halfFieldTag, tag2Center[2]),
          cv::Point3f(tag2Center[0] + halfFieldTag, tag2Center[1] + halfFieldTag, tag2Center[2]),
          cv::Point3f(tag2Center[0] + halfFieldTag, tag2Center[1] - halfFieldTag, tag2Center[2]),
          cv::Point3f(tag2Center[0] - halfFieldTag, tag2Center[1] - halfFieldTag, tag2Center[2])}},
        {3,
         {cv::Point3f(tag3Center[0] - halfFieldTag, tag3Center[1] + halfFieldTag, tag3Center[2]),
          cv::Point3f(tag3Center[0] + halfFieldTag, tag3Center[1] + halfFieldTag, tag3Center[2]),
          cv::Point3f(tag3Center[0] + halfFieldTag, tag3Center[1] - halfFieldTag, tag3Center[2]),
          cv::Point3f(tag3Center[0] - halfFieldTag, tag3Center[1] - halfFieldTag, tag3Center[2])}},
        {4,
         {cv::Point3f(tag4Center[0] - halfFieldTag, tag4Center[1] + halfFieldTag, tag4Center[2]),
          cv::Point3f(tag4Center[0] + halfFieldTag, tag4Center[1] + halfFieldTag, tag4Center[2]),
          cv::Point3f(tag4Center[0] + halfFieldTag, tag4Center[1] - halfFieldTag, tag4Center[2]),
          cv::Point3f(tag4Center[0] - halfFieldTag, tag4Center[1] - halfFieldTag, tag4Center[2])}}
    };
    ImGui::TextUnformatted("System State:");
    ImGui::SameLine();
    if (ctx.isTrackingActive) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "ACTIVE");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "IDLE");
    }
    ImGui::TextUnformatted("Field Alignment Status:");
    ImGui::SameLine();
    if (isCameraLocked) {
        ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "LOCKED IN (Field tags ignored)");
    } else if (isFieldOriented) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "ALIGNED (Tracking field tags 1-4)");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "UNALIGNED (Field tags required)");
    }
    if (ctx.isTrackingActive) {
        if (isFieldOriented && !isCameraLocked) {
            if (ImGui::Button("Lock Camera Extrinsics")) {
                isCameraLocked = true;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(Safe to remove field tags after locking)");
        } else if (isCameraLocked) {
            if (ImGui::Button("Unlock / Recalibrate Camera")) {
                isCameraLocked = false;
                resetCalibrationRequested = true;
            }
        }
    }
    if (!ctx.isCalibrated) {
        ImGui::BeginDisabled();
        ctx.isTrackingActive = false;
    }
    if (ctx.isTrackingActive) {
        if (ImGui::Button("Stop Localization Pipeline")) {
            ctx.isTrackingActive = false;
            isFieldOriented = false;
            if (isLoggingActive) {
                logger.stop();
                isLoggingActive = false;
            }
        }
    } else {
        if (ImGui::Button("Start Localization Pipeline")) {
            ctx.isTrackingActive = true;
        }
    }
    ImGui::SameLine();
    if (!ctx.isTrackingActive) {
        ImGui::BeginDisabled();
    }
    if (isLoggingActive) {
        if (ImGui::Button("Stop Logging")) {
            logger.stop();
            isLoggingActive = false;
        }
    } else {
        if (ImGui::Button("Start Logging")) {
            logger.start(currentLogTarget, "robot_pose.log");
            isLoggingActive = true;
        }
    }
    ImGui::SameLine();
    if (isLoggingActive) {
        ImGui::BeginDisabled();
    }
    const char* logTargetNames[] = { "Console", "File", "Both" };
    int currentTargetIdx = static_cast<int>(currentLogTarget);
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::Combo("##LogMode", &currentTargetIdx, logTargetNames, IM_ARRAYSIZE(logTargetNames))) {
        currentLogTarget = static_cast<LogTarget>(currentTargetIdx);
    }
    if (isLoggingActive) {
        ImGui::EndDisabled();
    }
    if (!ctx.isTrackingActive) {
        ImGui::EndDisabled();
    }
    if (!ctx.isCalibrated) {
        ImGui::EndDisabled();
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                           "ERROR: Camera/Field system calibration required before execution.");
    }
    ImGui::Separator();
    std::map<int, bool> tagVisibility = {{0, false}, {1, false}, {2, false}, {3, false}, {4, false}};
    if (ctx.isTrackingActive) {
        cv::Mat frameCopy;
        {
            std::lock_guard<std::mutex> lock(ctx.frame_mutex);
            if (!ctx.shared_frame.empty()) {
                ctx.shared_frame.copyTo(frameCopy);
            }
        }
        if (!frameCopy.empty()) {
            computeLocalizationPose(frameCopy, cameraMatrix, distCoeffs,
                                    worldFieldMap, ctx.currentPose, tagVisibility, ctx);
            resetCalibrationRequested = false;
            if (isLoggingActive && isFieldOriented) {
                logger.logPose(ctx.currentPose);
            }
        }
        ImGui::Text("Fiducial Visibility Telemetry:");
        for (int targetId = 0; targetId <= 4; ++targetId) {
            std::string label = (targetId == 0) ? "Tag ID 0 (Robot Target): "
                                                : "Tag ID " + std::to_string(targetId) + " (Field Boundary): ";
            ImGui::Text("%s", label.c_str());
            ImGui::SameLine();
            if (tagVisibility[targetId]) {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "VISIBLE");
            } else if (targetId > 0 && isCameraLocked) {
                ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "IGNORED (LOCKED)");
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "MISSING / OCCLUDED");
            }
        }
        ImGui::Separator();
        if (!isFieldOriented) {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "CRITICAL ERROR: Bring environmental anchors in frame.");
        }
    }
    if (ctx.isTrackingActive && isFieldOriented) {
        ImGui::TextUnformatted("Calculated Robot Pose (Center Origin, Heading CCW+):");
        ImGui::Indent(10.0f);
        ImGui::Text("Field Coordinate X : %.2f inches", ctx.currentPose.x);
        ImGui::Text("Field Coordinate Y : %.2f inches", ctx.currentPose.y);
        ImGui::Text("Heading Orientation: %.1f degrees (0° = Up, CCW+)", ctx.currentPose.theta);
        ImGui::Unindent(10.0f);
    } else if (ctx.isTrackingActive) {
        ImGui::TextUnformatted("Telemetry Status: Parsing frame streams to find global matrix frame...");
    }
    ImGui::EndChild();
}