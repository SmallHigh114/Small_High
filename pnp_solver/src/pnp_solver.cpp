#include "../include/pnp_solver.hpp"

PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, 
  const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
}

void PnPSolver::calculatePoint(const LaserModule& LaserModule)
{
    // Unit: m
  constexpr double four_half_y = FOUR_LASER_WIDTH  / 2.0 / 1000.0;
  constexpr double four_half_z = FOUR_LASER_HEIGHT / 2.0 / 1000.0;
  constexpr double six_half_y  = FOUR_LASER_WIDTH  / 2.0 / 1000.0;
  constexpr double six_half_z  = FOUR_LASER_HEIGHT / 2.0 / 1000.0;

  if (lasermodule.type == LaserType::FOUR)
  {
    laser_points_.emplace_back(cv::Point3f(0, four_half_y, -four_half_z));
    laser_points_.emplace_back(cv::Point3f(0, four_half_y, four_half_z));
    laser_points_.emplace_back(cv::Point3f(0, -four_half_y, four_half_z));
    laser_points_.emplace_back(cv::Point3f(0, -four_half_y, -four_half_z));
  }
  else
  {
    laser_points_.emplace_back(cv::Point3f(0, four_half_y, -four_half_z));
    laser_points_.emplace_back(cv::Point3f(0, four_half_y, 0)); 
    laser_points_.emplace_back(cv::Point3f(0, four_half_y, four_half_z));
    laser_points_.emplace_back(cv::Point3f(0, -four_half_y, four_half_z));
    laser_points_.emplace_back(cv::Point3f(0, -four_half_y, 0));
    laser_points_.emplace_back(cv::Point3f(0, -four_half_y, -four_half_z));
  }
}

bool PnPSolver::solvePnP(const LaserModule& laserModule, cv::Mat& rvec, cv::Mat& tvec)
{
  std::vector<cv::Point2f> image_points;
  calculatePoint(lasermodule);
  // Store counterclockwise
  for (const auto& center : lasermodule.Centers)
  {
    image_points.emplace_back(center);
  }

  // Solve pnp
  return cv::solvePnP(
    laser_points_, image_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_EPNP);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}