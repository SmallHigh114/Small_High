#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

// STD
#include <array>
#include <vector>

#include "detector/detector.hpp"

class PnPSolver
{

public:
    PnPSolver(
        const cv::Mat& camera_matrix,
        const cv::Mat& distortion_coefficients);

    bool solvePnP(const LaserModule& lasermodule, cv::Mat& rvec, cv::Mat& tvec);

    float calculateDistanceToCenter(const cv::Point2f& image_point);

private:

    void calculatePoint(const LaserModule& lasermodule);

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Unit: mm
    static constexpr float SIX_LASER_WIDTH = 40;   // Width of a laser module with 6 lights
    static constexpr float SIX_LASER_HEIGHT = 50;  // Height of a laser module with 6 lights
    static constexpr float FOUR_LASER_WIDTH = 27;  // Width of a laser module with 4 lights
    static constexpr float FOUR_LASER_HEIGHT = 50; // Height of a laser module with 6 lights

    std::vector<cv::Point3f> laser_points_;
    LaserModule lasermodule;
};
#endif // PNP_SOLVER_HPP