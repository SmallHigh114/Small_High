#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

// STD
#include <array>
#include <vector>

#include "../../detector/include/laser.hpp"

class PnPSolver
{

public:
    PnPSolver(
        const std::array<double, 9>& camera_matrix,
        const std::vector<double>& distortion_coefficients);

    bool solvePnP(const LaserModule& laserModule, cv::Mat& rvec, cv::Mat& tvec);

    float calculateDistanceToCenter(const cv::Point2f& image_point);

private:

    void calculatePoint(const LaserModule& LaserModule);

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Unit: mm
    static constexpr float SIX_LASER_WIDTH = 40;
    static constexpr float SIX_LASER_HEIGHT = 50;
    static constexpr float FOUR_LASER_WIDTH = 27;
    static constexpr float FOUR_LASER_HEIGHT = 50;

    std::vector<cv::Point3f> laser_points_;
    LaserModule lasermodule;

};


#endif // PNP_SOLVER_HPP