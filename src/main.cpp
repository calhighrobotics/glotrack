// Copyright 2026 DataTopping
// SPDX-License-Identifier: GPL-3.0-or-later

#include "AppContext.hpp"
#include "LocalizationPipeline.hpp"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"
#include "implot.h"
#include "util.hpp"
#include <GLES3/gl3.h>
#include <GLFW/glfw3.h>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iostream>
#include <mutex>
#include <numbers>
#include <opencv2/calib.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/flann/dist.h>
#include <opencv2/geometry/3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include <thread>

static AppContext g_ctx;
static LocalizationPipeline g_localizationPipeline;
constexpr double RAD_TO_DEG = 180.0 * std::numbers::inv_pi_v<double>;

void renderCameraFocusWindow(cv::VideoCapture& cap, CameraFocusSettings& settings) {
    ImGui::Begin("Camera Focus Control");
    if (ImGui::Checkbox("Auto Focus", &settings.autoFocus)) {
        cap.set(cv::CAP_PROP_AUTOFOCUS, settings.autoFocus ? 1.0 : 0.0);
    }
    if (settings.autoFocus) {
        ImGui::BeginDisabled();
    }
    if (ImGui::SliderInt("Focus Value", &settings.focusValue, 0, settings.maxFocus)) {
        cap.set(cv::CAP_PROP_AUTOFOCUS, 0.0);
        settings.autoFocus = false;
        cap.set(cv::CAP_PROP_FOCUS, static_cast<double>(settings.focusValue));
    }
    if (settings.autoFocus) {
        ImGui::EndDisabled();
    }
    ImGui::Separator();
    double currentFocusReadback = cap.get(cv::CAP_PROP_FOCUS);
    ImGui::Text("Hardware Focus Readback: %.0f", currentFocusReadback);
    ImGui::End();
}

void renderFieldPlotWindow(const RobotPose2D& currentPose, bool isTrackingActive) {
    static GLuint fieldTexture = 0;
    static bool textureLoaded = false;
    if (!textureLoaded) {
        fieldTexture = loadTextureFromFile("img/override_field.png");
        textureLoaded = true;
    }

    static std::deque<RobotPose2D> poseHistory;
    constexpr size_t MAX_TRAIL_POINTS = 150;

    if (isTrackingActive) {
        bool shouldAppend = poseHistory.empty();
        if (!shouldAppend) {
            const double dx = currentPose.x - poseHistory.back().x;
            const double dy = currentPose.y - poseHistory.back().y;

            if ((dx * dx + dy * dy) > 0.01) {
                shouldAppend = true;
            }
        }

        if (shouldAppend) {
            poseHistory.push_back(currentPose);
            if (poseHistory.size() > MAX_TRAIL_POINTS) {
                poseHistory.pop_front();
            }
        }
    }

    if (ImGui::Button("Clear Motion Trail")) {
        poseHistory.clear();
    }

    if (ImPlot::BeginPlot("Field View", ImVec2(-1, -1), ImPlotFlags_Equal)) {
        ImPlot::SetupAxes("Field X (Inches)", "Field Y (Inches)");
        ImPlot::SetupAxesLimits(-75.0, 75.0, -75.0, 75.0, ImGuiCond_FirstUseEver);

        if (fieldTexture != 0) {
            ImPlot::PlotImage("Field Overhead", (ImTextureID)(intptr_t)fieldTexture, ImVec2(-75.0f, -75.0f),
                              ImVec2(75.0f, 75.0f));
        }

        const size_t nPoints = poseHistory.size();
        if (nPoints > 1) {

            struct TrailData {
                const std::deque<RobotPose2D>* history;
            } dataData{&poseHistory};

            auto PointGetter = [](int idx, void* user_data) -> ImPlotPoint {
                const auto* data = static_cast<TrailData*>(user_data);
                const auto& pose = (*data->history)[idx];
                return ImPlotPoint(pose.x, pose.y);
            };
            ImDrawList* drawList = ImPlot::GetPlotDrawList();
            ImPlot::PushPlotClipRect();

            for (size_t i = 0; i < nPoints - 1; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(nPoints - 1);

                const ImU32 col = ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f - t, t, 0.0f, 0.3f + 0.7f * t));

                ImVec2 p1 = ImPlot::PlotToPixels(ImPlotPoint(poseHistory[i].x, poseHistory[i].y));
                ImVec2 p2 = ImPlot::PlotToPixels(ImPlotPoint(poseHistory[i + 1].x, poseHistory[i + 1].y));

                drawList->AddLine(p1, p2, col, 3.0f);
            }

            ImPlot::PopPlotClipRect();
        }

        if (isTrackingActive) {
            constexpr double hw = 14.0 / 2.0;
            constexpr double hl = 16.0 / 2.0;

            const double rad = currentPose.theta * (std::numbers::pi / 180.0);
            const double cosT = std::cos(rad);
            const double sinT = std::sin(rad);

            const double localX[5] = {-hw, hw, hw, -hw, -hw};
            const double localY[5] = {-hl, -hl, hl, hl, -hl};

            double worldX[5], worldY[5];
            for (int i = 0; i < 5; ++i) {
                worldX[i] = currentPose.x + (localX[i] * cosT + localY[i] * sinT);
                worldY[i] = currentPose.y + (-localX[i] * sinT + localY[i] * cosT);
            }

            ImPlotSpec fillSpec;
            fillSpec.FillColor = ImVec4(0.0f, 0.7f, 1.0f, 0.4f);
            ImPlot::PlotPolygon("##RobotBody", worldX, worldY, 5, fillSpec);
            ImPlotSpec outlineSpec;
            outlineSpec.LineColor = ImVec4(0.0f, 0.9f, 1.0f, 1.0f);
            outlineSpec.LineWeight = 2.5f;
            ImPlot::PlotLine("##RobotOutline", worldX, worldY, 5, outlineSpec);
            const double frontX[2] = {currentPose.x, currentPose.x + sinT * (hl + 6.0)};
            const double frontY[2] = {currentPose.y, currentPose.y + cosT * (hl + 6.0)};

            ImPlotSpec headingSpec;
            headingSpec.LineColor = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            headingSpec.LineWeight = 3.0f;
            ImPlot::PlotLine("##HeadingIndicator", frontX, frontY, 2, headingSpec);
        }

        ImPlot::EndPlot();
    }
}

// Worker thread function to continuously update the camera frame in the AppContext
void cameraWorker() {
    g_ctx.cap.open(g_ctx.cameraIndex, cv::CAP_ANY);
    if (!g_ctx.cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return;
    }
    int selectedCamIndex = g_ctx.cameraIndex;
    cv::Mat local_frame;
    while (g_ctx.camera_running) {
        if (selectedCamIndex != g_ctx.cameraIndex) {
            g_ctx.cap.release();
            g_ctx.cap.open(g_ctx.cameraIndex, cv::CAP_ANY);
            if (!g_ctx.cap.isOpened()) {
                g_ctx.cameraIndex = selectedCamIndex;
                continue;
            }
            selectedCamIndex = g_ctx.cameraIndex;
        }
        if (g_ctx.cap.read(local_frame) && !local_frame.empty()) {
            std::lock_guard<std::mutex> lock(g_ctx.frame_mutex);
            local_frame.copyTo(g_ctx.shared_frame);
        }
    }
    g_ctx.cap.release();
    return;
}

// Draws the calibration UI window, allowing users calibrate and compute the camera's intrinsic matrix.
void drawCalibrationUI(GLuint textureId, float childHeight) {
    static cv::Size boardSize(9, 6);
    static float squareSize = 1.0f;
    static std::vector<std::vector<cv::Point2f>> allImagePoints;
    static std::vector<float> perViewErrors;
    static double totalAvgError = 0.0;
    static std::string statusMessage = "Ready to capture.";

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(g_ctx.frame_mutex);
        if (!g_ctx.shared_frame.empty()) {
            g_ctx.shared_frame.copyTo(frame);
        }
    }

    ImGui::Begin("Camera Calibration");

    if (ImGui::InputInt("Grid Width", &boardSize.width)) {
        boardSize.width = std::max(2, boardSize.width);
    }
    if (ImGui::InputInt("Grid Height", &boardSize.height)) {
        boardSize.height = std::max(2, boardSize.height);
    }
    ImGui::InputFloat("Square Size", &squareSize, 0.1f, 1.0f, "%.2f");

    ImGui::Text("Collected Snapshots: %d", static_cast<int>(allImagePoints.size()));
    ImGui::TextWrapped("Status: %s", statusMessage.c_str());

    if (!allImagePoints.empty() && ImGui::Button("Reset / Clear Snapshots")) {
        allImagePoints.clear();
        perViewErrors.clear();
        g_ctx.isCalibrated = false;
        totalAvgError = 0.0;
        statusMessage = "Dataset cleared. Ready to capture fresh snapshots.";
    }

    ImGui::Separator();

    if (ImGui::Button("Capture Current View", ImVec2(-1, 0))) {
        std::vector<cv::Point2f> corners;
        int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

        bool found = cv::findChessboardCorners(frame, boardSize, corners, flags);

        if (found) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));

            allImagePoints.push_back(corners);
            statusMessage = "Successfully captured snapshot " + std::to_string(allImagePoints.size()) + "!";
        } else {
            statusMessage = "Error: Chessboard not found. Check pattern dimensions or lighting.";
        }
    }

    if (allImagePoints.size() >= 3 && ImGui::Button("Calculate Intrinsic Matrix", ImVec2(-1, 0))) {
        std::vector<cv::Mat> rvecs, tvecs;

        std::vector<cv::Point3f> obj;
        for (int i = 0; i < boardSize.height; ++i) {
            for (int j = 0; j < boardSize.width; ++j) {
                obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
            }
        }

        std::vector<std::vector<cv::Point3f>> allObjPoints(allImagePoints.size(), obj);

        g_ctx.K = cv::Mat::eye(3, 3, CV_64F);
        g_ctx.D = cv::Mat::zeros(8, 1, CV_64F);

        double rms = cv::calibrateCamera(allObjPoints, allImagePoints, frame.size(), g_ctx.K, g_ctx.D, rvecs, tvecs);

        bool isNumericallyValid = cv::checkRange(g_ctx.K) && cv::checkRange(g_ctx.D);

        totalAvgError =
            computeReprojectionErrors(allObjPoints, allImagePoints, rvecs, tvecs, g_ctx.K, g_ctx.D, perViewErrors);
        if (isNumericallyValid && totalAvgError < 2.0) {
            saveCameraCalibration("intrinsic_camera_matrices.json", g_ctx.K, g_ctx.D);
            g_ctx.isCalibrated = true;
            statusMessage = cv::format("Calibration Succeeded! RMS: %.3f px, MRE: %.3f px", rms, totalAvgError);
        } else {
            g_ctx.isCalibrated = false;
            statusMessage = cv::format(
                "Calibration Failed Quality Check! MRE: %.3f px (Too high. Clear and retry with better angles).",
                totalAvgError);
        }
    }

    ImGui::Separator();

    if (g_ctx.isCalibrated) {

        if (totalAvgError < 0.5) {
            ImGui::TextColored(ImVec4(0, 1, 0, 1), "Quality: EXCELLENT (%.3f px)", totalAvgError);
        } else if (totalAvgError < 1.0) {
            ImGui::TextColored(ImVec4(0.8f, 1, 0, 1), "Quality: GOOD (%.3f px)", totalAvgError);
        } else {
            ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "Quality: MARGINAL (%.3f px)", totalAvgError);
        }

        if (ImGui::TreeNode("Calibration Matrices")) {
            std::stringstream k_str, d_str;
            k_str << g_ctx.K;
            d_str << g_ctx.D;
            ImGui::Text("Camera Matrix (K):\n%s", k_str.str().c_str());
            ImGui::Text("Distortion Coefficients (D):\n%s", d_str.str().c_str());
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Per-View Reprojection Errors")) {
            for (size_t i = 0; i < perViewErrors.size(); ++i) {
                ImGui::Text("Snapshot %zu Error: %.3f px", i + 1, perViewErrors[i]);
            }
            ImGui::TreePop();
        }
    }
    ImGui::End();
}

int main(int argc, char** argv) {
    std::thread cam_thread(cameraWorker);
    if (loadCameraCalibration("intrinsic_camera_matrices.json", g_ctx.K, g_ctx.D)) {
        g_ctx.isCalibrated = true;
    } else {
        std::cout << "[INFO] No calibration file found. Please calibrate the camera first." << std::endl;
    }

    // General boilerplate for setting up OpenGL and ImGui
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
    GLFWwindow* window =
        glfwCreateWindow((int)(1280 * main_scale), (int)(800 * main_scale), "Glotrack", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.Fonts->AddFontDefaultVector();
    io.Fonts->AddFontDefaultBitmap();
    io.Fonts->AddFontDefault();
    io.Fonts->AddFontFromFileTTF("fonts/RobotoMono-Regular.ttf");

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);
    style.FontScaleDpi = main_scale;
    style.FontSizeBase = 20.0f;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    const char* glsl_version = nullptr;
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Setup OpenGL texture for video feed
    GLuint videoTexture;
    glGenTextures(1, &videoTexture);

    // Main application loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        auto io = ImGui::GetIO();
        auto viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);

        // Setup fullscreen window flags
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |

                                        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        ImGui::Begin("Main Window", nullptr, window_flags);

        // Left graph window for field plot
        {
            ImGui::BeginChild("Localization Plot", ImVec2(ImGui::GetContentRegionAvail().x * 0.6f, 0.0f),
                              ImGuiChildFlags_Borders);
            renderFieldPlotWindow(g_ctx.currentPose, g_ctx.isTrackingActive);
            ImGui::EndChild();
        }

        ImGui::SameLine();

        // Right side window for live video stream, calibration, and localization controls
        {
            ImGui::BeginGroup();
            float childHeight = ImGui::GetContentRegionAvail().y * 0.5f;
            ImGui::BeginChild("Live Video Stream", ImVec2(ImGui::GetContentRegionAvail().x, childHeight),
                              ImGuiChildFlags_Borders);

            ImGui::Text("Select Feed:");
            ImGui::SameLine();
            if (ImGui::RadioButton("Camera 1", g_ctx.cameraIndex == 0)) {
                g_ctx.cameraIndex = 0;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Camera 2", g_ctx.cameraIndex == 1)) {
                g_ctx.cameraIndex = 1;
            }
            ImGui::SameLine();
            ImGui::Checkbox("Toggle Calibration Window", &g_ctx.showCalibrationWindow);

            ImGui::Separator();
            {
                std::lock_guard<std::mutex> lock(g_ctx.frame_mutex);
                if (!g_ctx.shared_frame.empty()) {
                    matToTexture(g_ctx.shared_frame, videoTexture);
                    ImVec2 availSize = ImGui::GetContentRegionAvail();
                    float frameAspect =
                        static_cast<float>(g_ctx.shared_frame.cols) / static_cast<float>(g_ctx.shared_frame.rows);
                    float availAspect = availSize.x / availSize.y;

                    ImVec2 renderSize;
                    if (frameAspect > availAspect) {
                        renderSize.x = availSize.x;
                        renderSize.y = availSize.x / frameAspect;
                    } else {
                        renderSize.y = availSize.y;
                        renderSize.x = availSize.y * frameAspect;
                    }

                    ImGui::Image((ImTextureID)(intptr_t)videoTexture, renderSize);
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "No video feed available.");
                }
            }

            if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Q, ImGuiInputFlags_RouteGlobal)) {
                g_ctx.camera_running = false;
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }

            ImGui::EndChild();
            if (!g_ctx.isCalibrated || g_ctx.showCalibrationWindow) {
                drawCalibrationUI(videoTexture, childHeight);
            }
            g_localizationPipeline.renderLocalizationWindow(g_ctx, g_ctx.K, g_ctx.D);
            ImGui::EndGroup();
        }

        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w,
                     clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
    cam_thread.join();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}