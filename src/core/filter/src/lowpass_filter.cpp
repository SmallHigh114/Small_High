#include "filter/lowpass_filter.hpp"

AdaptiveLowPassFilter::AdaptiveLowPassFilter(const FilterParam& fp)
    : fp_(fp), alpha_current_(fp.alpha_static), initialized_(false) 
{
}

LaserModule AdaptiveLowPassFilter::apply(const LaserModule& laserModule)
{
    const std::vector<cv::Point2f>& centers = laserModule.Centers;
    const size_t num = centers.size();
    if (centers.empty()) return laserModule;

    if (initialized_ && prev_.size() != num)
    {
        initialized_ = false;
    }

    if (!initialized_) 
    {
        prev_ = centers;
        initialized_ = true;
        return laserModule;
    }

    float max_disp = 0.f;
    for (size_t i = 0; i < num; ++i) 
    {
        float dx = centers[i].x - prev_[i].x;
        float dy = centers[i].y - prev_[i].y;
        max_disp = std::max(max_disp, std::sqrt(dx*dx + dy*dy));
    }
    float alpha_target = (max_disp > fp_.motion_thresh) ? fp_.alpha_dynamic : fp_.alpha_static;

    alpha_current_ += fp_.smooth_rate * (alpha_target - alpha_current_);

    const float a = alpha_current_;
    const float a_t = 2.f * a / (1.f + a);   // Tustin 修正

    std::vector<cv::Point2f> smoothed(num);
    for (size_t i = 0; i < num; ++i) 
    {
        smoothed[i].x = a_t * centers[i].x + (1.f - a_t) * prev_[i].x;
        smoothed[i].y = a_t * centers[i].y + (1.f - a_t) * prev_[i].y;
    }

    prev_ = smoothed;
    LaserModule result = laserModule;
    result.Centers = smoothed;
    return result;
}

void AdaptiveLowPassFilter::reset() 
{ 
    initialized_ = false;
    alpha_current_ = fp_.alpha_static;
}