#include "detector/laser.hpp"
class ExponentialFilter {
public:
    explicit ExponentialFilter(float alpha = 0.3f)
        : Alpha(alpha), initialized_(false) {}

    // 每帧调用一次，传入当前帧的 LaserModule
    LaserModule apply(const LaserModule& laserModule)
    {
        const std::vector<cv::Point2f>& centers = laserModule.Centers;
        if (centers.empty()) return laserModule;

        // 首帧初始化历史状态
        if (!initialized_ || prevSmoothed_.size() != centers.size()) {
            prevSmoothed_ = centers;
            initialized_ = true;
            return laserModule;
        }

        // 对每个点独立做跨帧平滑
        std::vector<cv::Point2f> smoothed(centers.size());
        for (size_t i = 0; i < centers.size(); ++i) {
            smoothed[i] = exponentialSmoothing(prevSmoothed_[i], centers[i], Alpha);
        }

        prevSmoothed_ = smoothed;  // 保存本帧结果，供下帧使用

        LaserModule result = laserModule;
        result.Centers = smoothed; // 写回
        return result;
    }

private:
    cv::Point2f exponentialSmoothing(cv::Point2f prev, cv::Point2f cur, float alpha)
    {
        return cv::Point2f(
            alpha * cur.x + (1.0f - alpha) * prev.x,
            alpha * cur.y + (1.0f - alpha) * prev.y
        );
    }

    float Alpha;                            // 一次指数平滑参数
    std::vector<cv::Point2f> prevSmoothed_; // 历史平滑值
    bool initialized_;                      // 初始化
};