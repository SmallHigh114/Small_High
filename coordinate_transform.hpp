// 坐标系转化 相机光心 → 相机 → 云台 → Odom(世界坐标系)
// 代码全部以右手坐标系为准
// 电控传入的yaw值默认右手坐标系下的坐标轴指向为正值

#ifndef COORDINATE_TRANSFORM_HPP
#define COORDINATE_TRANSFORM_HPP

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
 * @brief 坐标系转换
 * @param rvec          输入点的旋转向量
 * @param tvec          输入点的位置向量
 * @param gimbal_pitch  云台pitch角
 * @param cam2gimDis    相机光心到云台距离
 * @param odom_pitch    云台到世界坐标系pitch角
 * @param gim2odomDis   云台到世界坐标系距离
 * @param odom_tvec     返回Odom坐标系下的位置
 * @param odom_rvec     范围Odom坐标系下的姿态
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
    cv::Mat & odom_tvec,
    cv::Mat & odom_rvec,
    const float & gimbal_yaw = 0,
    const float & roll = 0
)
{
    Eigen::Vector3d P_photocenter;
    P_photocenter << tvec.at<double>(0),
                        tvec.at<double>(1),
                        tvec.at<double>(2);
    // camera photocenter coordinate system to camera coordinate system
    Eigen::Matrix3d R_camera_photocenter;
    R_camera_photocenter << 
        0,  0,  1,
        -1,  0,  0,
        0, -1,  0;
    Eigen::Vector3d P_camera =
    R_camera_photocenter * P_photocenter;
    
    // camera coordinate system to gimbal coordinate system
    Eigen::Matrix3d R_gimbal_camera;
    R_gimbal_camera <<
        cos(gimbal_yaw), -sin(gimbal_yaw), 0,
        sin(gimbal_yaw), cos(gimbal_yaw), 0,
        0, 0, 1;

    // Calculate t
    Eigen::Vector3d t_gimbal;
    t_gimbal << 1, 1, 1;
    float dis_cam = std::sqrt((t_gimbal(0) * t_gimbal(0)) + (t_gimbal(1) * t_gimbal(1)));
    
    if (gimbal_yaw != 0) {
        float angle_new;
        float angle_cam = std::atan2(t_gimbal(1), t_gimbal(0));
        if (gimbal_yaw > 0) {
            angle_new = angle_cam + gimbal_yaw;
        }
        else{
            angle_new = angle_cam - gimbal_yaw;
        }
        t_gimbal(0) = dis_cam * std::cos(angle_new);
        t_gimbal(1) = dis_cam * std::sin(angle_new);
    }

    float angle_cam = std::atan2(t_gimbal(1), t_gimbal(0));
    t_gimbal(2) = t_gimbal(2) +  cam2gimDis * sin(gimbal_pitch);
    float dis_xoy = cam2gimDis * cos(gimbal_pitch);
    t_gimbal(0) = t_gimbal(0) + dis_xoy * std::cos(angle_cam);
    t_gimbal(1) = t_gimbal(1) + dis_xoy * std::sin(angle_cam);
    
    Eigen::Vector3d P_gimbal = 
        R_gimbal_camera * P_camera + t_gimbal;

    // gimbal coordinate system to odom coordinate system
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_odom;
    t_odom(2) = gim2odomDis * std::sin(odom_pitch);
    t_odom(0) = gim2odomDis * std::cos(odom_pitch);
    t_odom(1) = 0;

    Eigen::Vector3d P_odom = 
        I * P_gimbal + t_odom;

    
    odom_tvec.at<double>(0,0) = P_odom(0);
    odom_tvec.at<double>(1,0) = P_odom(1);
    odom_tvec.at<double>(2,0) = P_odom(2);
    
    // Transform rvec
    cv::Mat R_cam_obj_cv;
    cv::Rodrigues(rvec, R_cam_obj_cv);

    Eigen::Matrix3d R_cam_obj;
    for (int i = 0; i < 3; i++) 
    {        
        for (int j = 0; j < 3; j++) 
        {
            R_cam_obj(i, j) = R_cam_obj_cv.at<double>(i, j);
        }
    }
    Eigen::Matrix3d R_total = R_gimbal_camera * R_camera_photocenter;
    Eigen::Matrix3d R_odom_obj = R_total * R_cam_obj;

    if (roll != 0)
    {
        Eigen::Matrix3d add_roll;
        add_roll <<
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll),  cos(roll);
        R_odom_obj *= add_roll;
    }

    cv::Mat R_odom_obj_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            R_odom_obj_cv.at<double>(i, j) = R_odom_obj(i, j);
        }
    }
    cv::Rodrigues(R_odom_obj_cv, odom_rvec);
}

}
#endif // COORDINATE_TRANSFORM_HPP
/**
//  * @brief 几何转化，貌似不太对
//  */
// cv::Mat tvecTransform(
//     const cv::Mat & tvec, 
//     const float & gimbal_pitch,
//     const float & cam2gimDis,
//     const float & odom_pitch,
//     const float & gim2odomDis,
//     const float & gimbal_yaw = 0,
//     const float & roll = 0
// )
// {
//     cv::Mat odom_tvec;
//     Eigen::Vector3d optical_center;
//     optical_center << tvec.at<double>(0),
//                         tvec.at<double>(1),
//                         tvec.at<double>(2);
//     camera_coordinate(0) = optical_center(2); // x
//     camera_coordinate(1) = -optical_center(0); // y
//     camera_coordinate(2) = -optical_center(1);  // z

//     // cameara coordinate to gimbal coordinate
//     float dis_cam = std::sqrt((camera_coordinate(0) * camera_coordinate(0)) + (camera_coordinate(1) * camera_coordinate(1)));
    
//     if (gimbal_yaw != 0) {
//         float angle_new;
//         float angle_cam = std::atan2(camera_coordinate(1), camera_coordinate(0));
//         if (gimbal_yaw > 0) {
//             angle_new = angle_cam + gimbal_yaw;
//         }
//         else{
//             angle_new = angle_cam - gimbal_yaw;
//         }
//         camera_coordinate(0) = dis_cam * std::cos(angle_new);
//         camera_coordinate(1) = dis_cam * std::sin(angle_new);
//     }

//     float angle_cam = std::atan2(camera_coordinate(1), camera_coordinate(0));
//     gimbal_coordinate(2) = camera_coordinate(2) +  cam2gimDis * sin(gimbal_pitch);
//     float dis_xoy = cam2gimDis * cos(gimbal_pitch);
//     gimbal_coordinate(0) = camera_coordinate(0) + dis_xoy * std::cos(angle_cam);
//     gimbal_coordinate(1) = camera_coordinate(1) + dis_xoy * std::sin(angle_cam);

//     // gimbal coordinate to odom coordinate
//     odom_coordinate(2) = gimbal_coordinate(2) + gim2odomDis * std::sin(odom_pitch);
//     odom_coordinate(0) = gimbal_coordinate(0) + gim2odomDis * std::cos(odom_pitch);
//     odom_coordinate(1) = gimbal_coordinate(1);

//     cv::eigen2cv(odom_coordinate, odom_tvec);

//     return odom_tvec;
// }

// Eigen::Vector3d camera_coordinate; // 相机坐标系
// Eigen::Vector3d gimbal_coordinate; // 云台坐标系
// Eigen::Vector3d odom_coordinate;   // Odom坐标系