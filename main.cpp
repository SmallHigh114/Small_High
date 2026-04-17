#include <iostream>
#include <opencv2/opencv.hpp>

#include "HikCamera/include/HikCamera.h"

#include "detector/include/laser.hpp"
#include "detector/include/detector.hpp"
#include "filter/include/lowpass_filter.hpp"
#include "pnp_solver/include/pnp_solver.hpp"

#include "UI/include/draw_ui.hpp"
#include "UI/include/capLog.hpp"

// #define DEBUG

int main() {
	#ifdef DEBUG
	debug::Logger logger("output");
	#endif // DEBUG
	cv::Mat frame;
	cv::Mat rvec;
	cv::Mat tvec;
	// std::array<double, 9> camera_matrix = 
	// {
	// };
    // std::vector<double> distortion_coefficients = {};
	Detector::LaserParam laserparam;
	laserparam.max_error = 10.00;
	laserparam.min_Ratio = 0.8;

	AdaptiveLowPassFilter::FilterParam fp;
	fp.alpha_static = 0.1;
	fp.alpha_dynamic = 0.8;
	fp.motion_thresh = 2;
	fp.smooth_rate = 0.15;

	YOLO yolo("../module/classes.txt", "../module/module.onnx", 0.5);
	Detector detector(88, 0, laserparam);
	AdaptiveLowPassFilter ALPfilter(fp);
	// PnPSolver pnpsolver(camera_matrix,distortion_coefficients);
	UI ui;

	// 相机
	// HikCamera camera(0);

	// cv::VideoCapture camera("../Video_20260315182119088.avi");
	cv::VideoCapture camera("../Video_20260315182913969.avi");
	// cv::VideoCapture camera("../60s.mp4");

	if (!camera.isOpened())
	{
		std::cerr << "Unable to open camera" << std::endl;
		return -1;
	}

	while (true)
	{
		camera >> frame;
		if (frame.empty())
		{
			std::cout << "Frame is empty" << std::endl;
			break;
		}

		std::vector<Detection> results = yolo.detect(frame);
		LaserModule laserModule = detector.lasermoduleDetector(frame, results);
		laserModule = ALPfilter.apply(laserModule); // 一阶低通滤波
		// pnpsolver.solvePnP(laserModule,rvec,tvec);
		// pnpsolver.calculateDistanceToCenter(laserModule.center);
		
		ui.drawFps(frame);
		ui.drawResult(frame, laserModule);
		ui.drawWaveforms(laserModule.rect, laserModule);
		// ui.drawDetections(frame, results);
		cv::imshow("Test Results", frame);

		if (cv::waitKey(1) == 27) break;
	}
	// camera.close();
	cv::destroyAllWindows();
}