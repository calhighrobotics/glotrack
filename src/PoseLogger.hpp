#pragma once

#include <string>
#include <string_view>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>

struct RobotPose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

enum class LogTarget {
    Console,
    File,
    Both
};

class PoseLogger {
public:
    PoseLogger() = default;
    ~PoseLogger();

    PoseLogger(const PoseLogger&) = delete;
    PoseLogger& operator=(const PoseLogger&) = delete;
    PoseLogger(PoseLogger&&) = delete;
    PoseLogger& operator=(PoseLogger&&) = delete;

    void start(LogTarget target = LogTarget::Console, std::string_view filename = "robot_pose.log");
    void stop();
    void logPose(const RobotPose2D& pose);

private:
    void workerLoop(this PoseLogger& self, std::stop_token stopToken);

    LogTarget m_target{LogTarget::Console};
    std::ofstream m_fileStream;
    std::queue<std::string> m_queue;
    std::mutex m_mutex;
    std::condition_variable_any m_cv;
    std::jthread m_workerThread;
};