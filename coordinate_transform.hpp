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
 * @param tvec 输入初始平移向量
 * @param rvec 输入初始旋转向量
 * @param cam2gim_angle 相机坐标系到云台坐标系连杆与水平面夹角
 * @param cam2gimDis 相机坐标系原点到云台坐标系原点距离
 * @param gim2odom_angle 云台坐标系到世界坐标系连杆与水平面夹角
 * @param gim2odomDis 云台坐标系原点到世界坐标原点距离
 * @param gimbal_pitch 云台pitch角
 * @param gimbal_yaw 云台yaw角
 * @param roll 世界坐标系下车辆倾斜
 */
void coordinateTransform(
    const cv::Mat & tvec, 
    const cv::Mat & rvec,
    const float & cam2gim_angle,
    const float & cam2gimDis,
    const float & gim2odom_angle,
    const float & gim2odomDis,
    const float & gimbal_pitch,
    const float & gimbal_yaw,
    cv::Mat & odom_tvec, 
    cv::Mat & odom_rvec,
    const float & gimbal_yaw_mis = 0,
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
    std::cout << "P_camera" << std::endl << P_camera << std::endl;

    /// camera coordinate system to gimbal coordinate system
    Eigen::Matrix3d R_gimbal_camera;
    R_gimbal_camera <<
        cos(gimbal_yaw_mis), -sin(gimbal_yaw_mis), 0,
        sin(gimbal_yaw_mis), cos(gimbal_yaw_mis), 0,
        0, 0, 1;
    Eigen::Vector3d t_gimbal;
    t_gimbal << cam2gimDis * cos(cam2gim_angle), 0, cam2gimDis * sin(cam2gim_angle);

    Eigen::Vector3d P_gimbal = 
    R_gimbal_camera * P_camera + t_gimbal;
    std::cout << "P_gimbal" << std::endl << P_gimbal << std::endl;

    /// gimbal coordinate system to odom coordinate system
    Eigen::Matrix3d R_odom_gimbal;
    R_odom_gimbal <<
    cos(gimbal_yaw)*cos(gimbal_pitch),
    cos(gimbal_yaw)*sin(gimbal_pitch)*sin(roll) - sin(gimbal_yaw)*cos(roll),
    cos(gimbal_yaw)*sin(gimbal_pitch)*cos(roll) + sin(gimbal_yaw)*sin(roll),
    sin(gimbal_yaw)*cos(gimbal_pitch),
    sin(gimbal_yaw)*sin(gimbal_pitch)*sin(roll) + cos(gimbal_yaw)*cos(roll),
    sin(gimbal_yaw)*sin(gimbal_pitch)*cos(roll) - cos(gimbal_yaw)*sin(roll),
    -sin(gimbal_pitch),
    cos(gimbal_pitch)*sin(roll),
    cos(gimbal_pitch)*cos(roll);

    Eigen::Vector3d t_odom;
    t_odom << gim2odomDis * cos(gim2odom_angle), 0, gim2odomDis * sin(gim2odom_angle);

    Eigen::Vector3d P_odom =
     R_odom_gimbal * P_gimbal + t_odom;
    std::cout << "P_odom"<< std::endl << P_odom << std::endl;

    odom_tvec = cv::Mat::zeros(3, 1, CV_64F);
    odom_tvec.at<double>(0,0) = P_odom(0);
    odom_tvec.at<double>(1,0) = P_odom(1);
    odom_tvec.at<double>(2,0) = P_odom(2);


    cv::Mat R_Mat;
    cv::Rodrigues(rvec, R_Mat);
    Eigen::Matrix3d R_cam_center;
    cv::cv2eigen(R_Mat, R_cam_center);

    Eigen::Matrix3d R = R_odom_gimbal * R_gimbal_camera * R_camera_photocenter * R_cam_center;
    cv::Mat R_mat_;
    cv::eigen2cv(R, R_mat_);
    cv::Rodrigues(R_mat_, odom_rvec);
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