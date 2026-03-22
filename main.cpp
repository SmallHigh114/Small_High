#include <iostream>
#include <opencv2/opencv.hpp>

#include "HikCamera/include/HikCamera.h"
#include "detector/include/laser.hpp"
#include "detector/include/detector.hpp"
#include "pnp_solver/include/pnp_solver.hpp"
#include "UI/include/draw_ui.hpp"

int main() {

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

	Detector detector(88, 0, laserparam);
	// PnPSolver pnpsolver(camera_matrix,distortion_coefficients);
	YOLO yolo("../module/classes.txt", "../module/module.onnx", 0.5);
	UI ui;

	// 相机
	// HikCamera camera(0);

	cv::VideoCapture camera("../Video_20260315182119088.avi");
	// cv::VideoCapture camera("../Video_20260315182913969.avi");

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