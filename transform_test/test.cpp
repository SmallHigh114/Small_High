#include <iostream>
#include <chrono>

#include "transform_coordinate/transform_coordinate.hpp"
#include "transform_coordinate/rvec_visualizer.hpp"

int main()
{
    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat odom_tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat odom_rvec = cv::Mat::zeros(3, 1, CV_64F);

    float cam2gimDis = 0.1;
    float gim2odom_angle = 1.57;
    float gim2odomDis = 0.5;
    float gimbal_pitch = -0.5236;
    float gimbal_yaw = -0.7854;
    float roll = 0;

    tvec.at<double>(0,0) = 0;
    tvec.at<double>(1,0) = 0;
    tvec.at<double>(2,0) = 2;
    rvec.at<double>(0,0) = 0;
    rvec.at<double>(1,0) = 0;
    rvec.at<double>(2,0) = 0;

    TransForm::coordinateTransform(
        tvec,
        rvec,
        cam2gimDis, 
        gim2odom_angle, 
        gim2odomDis, 
        gimbal_pitch, 
        gimbal_yaw,
        odom_tvec,
        odom_rvec,
        roll
    );

    std::cout << "odom_tvec = " << std::endl;
    std::cout << odom_tvec << std::endl;
    std::cout << "odom_rvec = " << std::endl;
    std::cout << odom_rvec << std::endl;

    auto end = std::chrono::high_resolution_clock::now();

    double duration_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    std::cout << "----------------------------------" << std::endl;
    std::cout << "Total time: " << duration_ms << " ms" << std::endl;

    visualizeRvec(odom_rvec);
    return 0;
}