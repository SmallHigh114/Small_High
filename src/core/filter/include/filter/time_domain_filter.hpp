#include "detector/laser.hpp"
#include <deque>

class TemporalFilter {
public:
    explicit TemporalFilter(int windowSize = 5)
        : windowSize_(windowSize) {}

    LaserModule apply(const LaserModule& laserModule)
    {
        const std::vector<cv::Point2f>& centers = laserModule.Centers;
        if (centers.empty()) return laserModule;

        // 点数变化时重置历史
        if (!history_.empty() && history_.front().size() != centers.size())
            history_.clear();

        // 压入当前帧，维护窗口大小
        history_.push_back(centers);
        if ((int)history_.size() > windowSize_)
            history_.pop_front();

        // 首帧直接返回
        if (history_.size() == 1) return laserModule;

        // 加权移动平均，越近的帧权重越高
        const size_t pointCount = centers.size();
        std::vector<cv::Point2f> smoothed(pointCount, {0.f, 0.f});

        float totalWeight = 0.f;
        int weight = 1;
        for (const auto& frame : history_)
        {
            for (size_t i = 0; i < pointCount; ++i)
            {
                smoothed[i].x += frame[i].x * weight;
                smoothed[i].y += frame[i].y * weight;
            }
            totalWeight += weight;
            weight++;   // 线性递增：最旧帧权重1，最新帧权重N
        }

        for (auto& p : smoothed)
        {
            p.x /= totalWeight;
            p.y /= totalWeight;
        }

        LaserModule result = laserModule;
        result.Centers = smoothed;
        return result;
    }

private:
    int windowSize_;                                // 历史帧数量
    std::deque<std::vector<cv::Point2f>> history_;  // 储存历史点集
};