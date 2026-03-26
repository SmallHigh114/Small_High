#include "../../detector/include/laser.hpp"

class AdaptiveLowPassFilter {
public:
    struct FilterParam {
        float alpha_static  = 0.1f;   // 静止状态参数
        float alpha_dynamic = 0.8f;   // 运动状态参数
        float motion_thresh = 2.0f;   // 运动判定阈值
        float smooth_rate   = 0.15f;  // alpha平滑过渡速率
    };

    AdaptiveLowPassFilter(const FilterParam& fp);

    LaserModule apply(const LaserModule& laserModule);

    void reset();

private:
    FilterParam fp_;                // 滤波参数
    float alpha_current_;           // 平滑强度
    bool initialized_;              // 初始化
    std::vector<cv::Point2f> prev_; // 上一帧状态
};