#include "pnp_solver/pnp_solver.hpp"

PnPSolver::PnPSolver(
  const cv::Mat & camera_matrix, 
  const cv::Mat & dist_coeffs)
: camera_matrix_(camera_matrix.reshape(1, 3).clone())
  // dist_coeffs_(dist_coeffs_.reshape(1,5).clone())
{
}

void PnPSolver::calculatePoint(const LaserModule& lasermodule)
{
  laser_points_.clear();
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

bool PnPSolver::solvePnP(const LaserModule& lasermodule, cv::Mat& rvec, cv::Mat& tvec)
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