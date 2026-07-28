#include "PoseLogger.hpp"

#include <print>
#include <format>
#include <utility>

PoseLogger::~PoseLogger() {
    stop();
}

void PoseLogger::start(LogTarget target, std::string_view filename) {
    stop();
    m_target = target;
    if (m_target == LogTarget::File || m_target == LogTarget::Both) {
        m_fileStream.open(std::string(filename), std::ios::out | std::ios::app);
    }
    m_workerThread = std::jthread([this](std::stop_token st) {
        workerLoop(st);
    });
}

void PoseLogger::stop() {
    if (m_workerThread.joinable()) {
        m_workerThread.request_stop();
        m_cv.notify_one();
        m_workerThread.join();
    }
    if (m_fileStream.is_open()) {
        m_fileStream.close();
    }
}

void PoseLogger::logPose(const RobotPose2D& pose) {
    std::string formatted = std::format("[POSE] x: {:.3f} | y: {:.3f} | theta: {:.3f}\n", 
                                        pose.x, pose.y, pose.theta);

    {
        std::lock_guard lock(m_mutex);
        m_queue.push(std::move(formatted));
    }
    m_cv.notify_one();
}

void PoseLogger::workerLoop(this PoseLogger& self, std::stop_token stopToken) {
    while (true) {
        std::string message;

        {
            std::unique_lock lock(self.m_mutex);
            self.m_cv.wait(lock, stopToken, [&self] { 
                return !self.m_queue.empty(); 
            });
            if (self.m_queue.empty() && stopToken.stop_requested()) {
                break;
            }
            if (self.m_queue.empty()) {
                continue;
            }
            message = std::move(self.m_queue.front());
            self.m_queue.pop();
        }

        if (self.m_target == LogTarget::Console || self.m_target == LogTarget::Both) {
            std::print("{}", message);
        }

        if ((self.m_target == LogTarget::File || self.m_target == LogTarget::Both) && self.m_fileStream.is_open()) {
            std::print(self.m_fileStream, "{}", message);
        }
    }
}