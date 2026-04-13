#ifndef TRANSFORM_COORDINATE_COORDINATE_TRANSFORM_HPP
#define TRANSFORM_COORDINATE_COORDINATE_TRANSFORM_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

struct cam2gimParam
{
    float distance;
    float roll;
    float pitch;
    float yaw;
};


class TransForm
{

public:

    Eigen::Vector3d coordinateTransform(const cv::Mat rvec, const cv::Mat& tvec, const cam2gimParam& cam2gimparam)
    {
        Eigen::Vector3d optical_center;
        optical_center << tvec.at<double>(0),
                          tvec.at<double>(1),
                          tvec.at<double>(2);
        camera_coordinate(0) = -optical_center(1);
        camera_coordinate(1) = -optical_center(2);
        camera_coordinate(2) = optical_center(0);
        

    }

    Eigen::Matrix3d rvec2Rmatrix(const cv::Mat & rvec)
    {
        cv::Mat R; 
        cv::Rodrigues(rvec, R);
        Eigen::Matrix3d R_; 
        cv::cv2eigen(R, R_);
        return R_;
    }


private:
    Eigen::Vector3d camera_coordinate; // 相机坐标系
    Eigen::Vector3d gimbal_coordinate; // 云台坐标系
    Eigen::Vector3d odom_coordinate;   // Odom坐标系
};


#endif // TRANSFORM_COORDINATE_COORDINATE_TRANSFORM_HPP