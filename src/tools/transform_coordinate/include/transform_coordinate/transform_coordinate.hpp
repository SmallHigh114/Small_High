// 坐标系转化 相机光心 → 相机 → 云台 → Odom(世界坐标系)
// 相机 云台 Odom坐标系 以右手坐标系为准

// Unit: radian
// Unit: meter

#ifndef COORDINATE_TRANSFORM_HPP
#define COORDINATE_TRANSFORM_HPP

#include <iostream>
#include <cmath>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace TransForm
{
/**
 * @brief 坐标系变换
 * 
 * @param tvec 输入初始平移向量
 * @param rvec 输入初始旋转向量
 * 
 * @param cam2gimDis 相机坐标系原点到云台坐标系原点距离
 * @param gim2odom_angle 云台坐标系到世界坐标系连杆与水平面夹角
 * @param gim2odomDis 云台坐标系原点到世界坐标原点距离
 * 
 * @param gimbal_pitch 云台pitch角
 * @param gimbal_yaw 云台yaw角
 * 
 * @param odom_tvec 输出Odom坐标系下的平移向量
 * @param odom_rvec 输出Odom坐标系下的旋转向量
 * 
 * @param roll 世界坐标系下车辆倾斜
 */
void coordinateTransform(
    const cv::Mat & tvec, 
    const cv::Mat & rvec,
    const float & cam2gimDis,
    const float & gim2odom_angle,
    const float & gim2odomDis,
    const float & gimbal_pitch,
    const float & gimbal_yaw,
    cv::Mat & odom_tvec, 
    cv::Mat & odom_rvec,
    const float & roll = 0
)
{
    // Translation vector transformation
    Eigen::Vector3d P_photocenter;
    P_photocenter << tvec.at<double>(0),
                        tvec.at<double>(1),
                        tvec.at<double>(2);
    /// camera photocenter coordinate system to camera coordinate system
    Eigen::Matrix3d R_camera_photocenter;
    R_camera_photocenter << 
        0,  0,  1,
        -1,  0,  0,
        0, -1,  0;
    Eigen::Vector3d P_camera =
    R_camera_photocenter * P_photocenter;
    // std::cout << "P_camera" << std::endl << P_camera << std::endl;

    /// camera coordinate system to gimbal coordinate system
    Eigen::Matrix3d R_yaw;
    R_yaw <<
        cos(gimbal_yaw), -sin(gimbal_yaw), 0,
        sin(gimbal_yaw), cos(gimbal_yaw), 0,
        0, 0, 1;
    Eigen::Matrix3d R_pitch;
    R_pitch <<
        cos(gimbal_pitch), 0, sin(gimbal_pitch),
        0, 1, 0,
        -sin(gimbal_pitch), 0, cos(gimbal_pitch);
    Eigen::Matrix3d R_roll;
    R_roll << 
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    Eigen::Matrix3d R_gimbal_camera;
    R_gimbal_camera = R_pitch;

    Eigen::Vector3d t_gimbal;
    t_gimbal << cam2gimDis * cos(gimbal_pitch), 0, cam2gimDis * sin(gimbal_pitch);
    // std::cout << "t_gimbal" << std::endl << t_gimbal << std::endl;
    Eigen::Vector3d P_gimbal = 
    R_gimbal_camera * P_camera + t_gimbal;
    // std::cout << "P_gimbal" << std::endl << P_gimbal << std::endl;

    /// gimbal coordinate system to odom coordinate system

    Eigen::Matrix3d R_odom_gimbal;
    R_odom_gimbal = R_yaw * R_roll;

    Eigen::Vector3d t_odom;
    t_odom << gim2odomDis * cos(gim2odom_angle), 0, gim2odomDis * sin(gim2odom_angle);
    // std::cout << "t_odom: " <<  t_odom << std::endl;
    Eigen::Vector3d P_odom =
     R_odom_gimbal * P_gimbal + t_odom;
    // std::cout << "P_odom"<< std::endl << P_odom << std::endl;

    odom_tvec = cv::Mat::zeros(3, 1, CV_64F);
    odom_tvec.at<double>(0,0) = P_odom(0);
    odom_tvec.at<double>(1,0) = P_odom(1);
    odom_tvec.at<double>(2,0) = P_odom(2);

    // Transform rotation vector
    cv::Mat R_Mat;
    cv::Rodrigues(rvec, R_Mat);
    Eigen::Matrix3d R_cam_center;
    cv::cv2eigen(R_Mat, R_cam_center);

    Eigen::Matrix3d R_cam_center_corrected =
    R_camera_photocenter *
    R_cam_center *
    R_camera_photocenter.transpose();

    Eigen::Matrix3d R = R_odom_gimbal * R_gimbal_camera * R_cam_center_corrected;
    cv::Mat R_mat_;
    cv::eigen2cv(R, R_mat_);
    cv::Rodrigues(R_mat_, odom_rvec);
}
}
#endif // COORDINATE_TRANSFORM_HPP