#ifndef DETECTOR_HPP
#define DETECTOR_HPP

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// STD
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

#include "detector/laser.hpp"
#include "yolo/yolov11.hpp"

class Detector 
{

public:

	struct LaserParam
	{
		float min_Ratio; // light ratio
		float max_error; // 任意两灯 x/y 坐标最大差值
	};

	/**
	 * @brief 构造函数
	 */
	Detector(const int& bin_thresh, const int& color, const LaserParam& lp);

	/**
	 * @brief 模块区域检测
	 */
	std::vector<cv::Mat> extractLaser(const cv::Mat& frame, std::vector<Detection>& results);

	/**
	 * @brief 图像处理
	 */
	cv::Mat preprocessImage(const cv::Mat& rgb_img);

	/**
	 * @brief 模块检测器
	 */
	LaserModule lasermoduleDetector(const cv::Mat& frame, std::vector<Detection> results);

private:

	/**
	 * @brief 灯中心点获取
	 */
	std::vector<cv::Point2f> getCenter(cv::Rect& min_rect);

	/**
	 * @brief 灯检测器
	 */
	std::vector<cv::Rect> lightExtractor(const cv::Mat& binary_img);

	/**
	 * @brief 获取激光模块发光区域
	 */
	cv::Rect laserArea(const cv::Mat& binary_image, const std::vector<cv::Rect>& rects);

	/**
	 * @brief 判断灯面积和比例是否符合条件
	 */
	bool isLightArea(const cv::Rect& rect, const float& ratio);

	/**
	 * @brief 判断是否为所需灯
	 */
	bool isLight(const cv::Rect& rect1, const cv::Rect& rect2);


	cv::Mat binary_img;                  // 二值图
	int binary_thresh;                   // 二值化阈值
	int detect_color;                    // 检测颜色

    float x_;                            // x偏移
	float y_;			                 // y偏移

	LaserParam lp;                       // 参数
	std::vector<cv::Point2f> Points;     // 灯块中心点
	std::vector<cv::Mat> Laser_Areas;    // 激光模块区域
	std::vector<cv::Rect> Rects;         // 灯
	cv::Rect min_Rect;					 // 最大外接矩形
	LaserModule lm;                      // 激光模块
};

#endif // DETECTOR_HPP