// 坐标系转化 相机光心 → 相机 → 云台 → Odom(世界坐标系)
// 代码全部以右手坐标系为准
// 电控传入的yaw值默认右手坐标系下的坐标轴指向为正值

#ifndef TRANSFORM_COORDINATE_COORDINATE_TRANSFORM_HPP
#define TRANSFORM_COORDINATE_COORDINATE_TRANSFORM_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <optional>
#include <opencv2/core/eigen.hpp>

namespace TransForm
{

/**
 * @brief 坐标系转换
 * @param rvec          输入点的旋转向量
 * @param tvec          输入点的位置向量
 * @param gimbal_pitch  云台pitch角
 * @param cam2gimDis    相机光心到云台距离
 * @param odom_pitch 云台到世界坐标系pitch角
 * @param gim2odomDis   云台到世界坐标系距离
 * @param gimbal_yaw    云台yaw角 默认为0
 * @param roll          世界坐标系下的roll角 默认为0
 */
void coordinateTransform(
    const cv::Mat & rvec,
    const cv::Mat & tvec, 
    const float & gimbal_pitch,
    const float & cam2gimDis,
    const float & odom_pitch,
    const float & gim2odomDis,
    const float & gimbal_yaw = 0,
    const float & roll = 0
)
{
    Eigen::Vector3d optical_center;
    optical_center << tvec.at<double>(0),
                        tvec.at<double>(1),
                        tvec.at<double>(2);
    camera_coordinate(0) = -optical_center(1); // x
    camera_coordinate(1) = -optical_center(2); // y
    camera_coordinate(2) = optical_center(0);  // z
    
    float angle = std::atan2(camera_coordinate(1) ,camera_coordinate(0));
    float gim_pitch_deg = gimbal_pitch * 180.0 / M_PI;
    float gim_yaw_deg = gimbal_yaw * 180.0 / M_PI;
    float distance = std::sqrt(camera_coordinate(0) * camera_coordinate(0) + camera_coordinate(1) * camera_coordinate(1));

    if (gimbal_yaw != 0 && gimbal_yaw > 0)
    {
        float dif_angle = angle - std::abs(gim_yaw_deg);
        camera_coordinate(0) = distance * sin(dif_angle);
        camera_coordinate(1) = distance * cos(dif_angle);
    } 
    else if (gimbal_yaw != 0 && gimbal_yaw < 0)
    {
        float fit_angle = angle + std::abs(gim_yaw_deg);
        camera_coordinate(0) = distance * sin(fit_angle);
        camera_coordinate(1) = distance * cos(fit_angle);
    }

    // camera to odom
    float dx_cg = cam2gimDis * cos(gim_pitch_deg) * cos(angle);
    float dy_cg = cam2gimDis * cos(gim_pitch_deg) * sin(angle);
    float dz_cg = cam2gimDis * sin(gim_pitch_deg);
    
    gimbal_coordinate(0) = camera_coordinate(0) + dx_cg;
    gimbal_coordinate(1) = camera_coordinate(1) + dy_cg;
    gimbal_coordinate(2) = camera_coordinate(2) + dz_cg;


    // gimbal to odom
    float gim_angle = std::atan2(gimbal_coordinate(1) ,gimbal_coordinate(0));
    float odom_pitch_deg = odom_pitch * 180.0 / M_PI;

    float dx_go = gim2odomDis * cos(odom_pitch_deg) * cos(gim_angle);
    float dy_go = gim2odomDis * cos(odom_pitch_deg) * sin(gim_angle);
    float dz_go = gim2odomDis * sin(odom_pitch_deg);
    
    odom_coordinate(0) = gimbal_coordinate(0) + dx_go;
    odom_coordinate(1) = gimbal_coordinate(1) + dy_go;
    odom_coordinate(2) = gimbal_coordinate(2) + dz_go;


    cv::Mat odom_tvec;
    cv::eigen2cv(odom_coordinate, odom_tvec);
}

Eigen::Matrix3d rvec2Rmatrix(const cv::Mat & rvec)
{
    cv::Mat R; 
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d R_; 
    cv::cv2eigen(R, R_);
    return R_;
}

Eigen::Vector3d camera_coordinate; // 相机坐标系
Eigen::Vector3d gimbal_coordinate; // 云台坐标系
Eigen::Vector3d odom_coordinate;   // Odom坐标系
}



#endif // TRANSFORM_COORDINATE_COORDINATE_TRANSFORM_HPP