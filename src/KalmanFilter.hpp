#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>

class KalmanFilter {
public:
    KalmanFilter(float processNoise = 0.1f, float measurementNoise = 0.25f);

    cv::Point2f predict();
    cv::Point2f update(float measuredX, float measuredY);
    void reset();
    bool initialized() const;
private:
    cv::KalmanFilter kf;
    bool isInitialized;
    std::chrono::high_resolution_clock::time_point lastTime;
};