#ifndef LASER_HPP
#define LASER_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

const int RED = 0;
const int BLUE = 1;

enum class LaserType { FOUR, SIX, INVALID };

struct Light
{
    Light() = default;
	cv::Point2f center; // 灯块中心点
	double area;        // 灯块面积
	int color;          // 灯块颜色
};

struct LaserModule
{
    LaserModule() = default;

    std::vector<cv::Point2f> Centers; // 灯的所有中心点
    std::vector<cv::Rect> rects;      // 激光模块 灯

    cv::Rect2f rect;                  // 激光模块最小外接矩形
    cv::Point2f center;               // 激光模块中心点
    LaserType type;                   // 激光模块类型
    float width;                      // 宽
    float height;                     // 高
};

#endif // LASER_HPP