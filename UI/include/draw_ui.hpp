#ifndef DRAW_UI_HPP
#define DRAW_UI_HPP

#include "../../detector/include/laser.hpp"
#include "../../YOLO/include/YOLOv11.hpp"
#include <deque>
#include <chrono>
#include <numeric>

class UI
{

public:

	void drawFps(cv::Mat& frame);

	/**
	 * @brief 在图片上绘制结果
	 *
	 * @param frame 绘制图像
	 * @param laserModule 激光模块
	 */
	void drawResult(cv::Mat& frame, LaserModule laserModule);

	/**
     * @brief 在图像上绘制检测结果
     *
     * @param image 输入输出图像
     * @param detections 检测结果
     */
    void drawDetections(cv::Mat& image, const std::vector<Detection>& detections);

	/**
	 * @brief 更新并绘制三路相对抖动波形图（合并在一个窗口）
	 *
	 * @param minRect      绿色框（min_Rect）
	 * @param laserModule  激光模块（含 Centers）
	 *
	 * 波形图1：绿色框 min_Rect 中心相对滑动均值的抖动偏差 (dx, dy) [像素]
	 * 波形图2：蓝色框 Centers-minAreaRect 中心相对滑动均值的抖动偏差 (dx, dy) [像素]
	 * 波形图3：绿色框与蓝色框之间的相互抖动偏差 (ddx, ddy) [像素]
	 *          = (green_dx − blue_dx, green_dy − blue_dy)
	 */
	void drawWaveforms(const cv::Rect& minRect, const LaserModule& laserModule);

private:

	// ---- 历史原始坐标队列（用于计算滑动均值） ----
	static constexpr int HISTORY_LEN = 300;  // 波形显示帧数
	static constexpr int SMOOTH_WIN  = 30;   // 滑动均值窗口（帧），用于去除整体位移趋势

	struct RawHistory {
		std::deque<float> x, y;   // 原始绝对坐标
	};

	RawHistory raw_green_;   // 绿色框中心原始坐标
	RawHistory raw_blue_;    // 蓝色框中心原始坐标

	// ---- 抖动偏差历史队列（波形显示用） ----
	struct JitterHistory {
		std::deque<float> dx, dy;   // 相对抖动偏差 [像素]
	};

	JitterHistory jitter_green_;     // 波形图1
	JitterHistory jitter_blue_;      // 波形图2
	JitterHistory jitter_relative_;  // 波形图3

	// ---- 辅助函数 ----

	/** 向原始历史推入新值，超长则弹出旧值 */
	void pushRaw(RawHistory& raw, float x, float y);

	/** 向抖动历史推入新值，超长则弹出旧值 */
	void pushJitter(JitterHistory& hist, float dx, float dy);

	/**
	 * @brief 计算当前帧相对于滑动均值的抖动偏差
	 * @param raw     含刚推入当前帧的原始历史
	 * @param out_dx  输出：x 方向抖动偏差 [像素]
	 * @param out_dy  输出：y 方向抖动偏差 [像素]
	 */
	void computeJitter(const RawHistory& raw, float& out_dx, float& out_dy);

	/**
	 * @brief 绘制单条波形子图
	 * @param canvas   目标绘制区域（子图 ROI，坐标从 (0,0) 开始）
	 * @param dxData   X 方向抖动历史
	 * @param dyData   Y 方向抖动历史
	 * @param title    子图标题
	 * @param colorDx  X 通道折线颜色
	 * @param colorDy  Y 通道折线颜色
	 */
	void drawSingleWaveform(
		cv::Mat& canvas,
		const std::deque<float>& dxData,
		const std::deque<float>& dyData,
		const std::string& title,
		const cv::Scalar& colorDx,
		const cv::Scalar& colorDy);
};

#endif // DRAW_UI_HPP