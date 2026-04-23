#include <iostream>
#include <opencv2/opencv.hpp>

#include "manage/Config.hpp"

#include "hikcamera/hikcamera.h"

#include "detector/laser.hpp"
#include "detector/detector.hpp"
#include "filter/lowpass_filter.hpp"
#include "pnp_solver/pnp_solver.hpp"
#include "transform_coordinate/transform_coordinate.hpp"

#include "ui/draw_ui.hpp"
#include "ui/capLog.hpp"

int main() 
{
	if (!J_COMMON.config_.isOpened())
    {
        std::cerr << "Failed to open config.yaml" << std::endl;
        return -1;
    }
	cv::Mat frame, rvec, tvec;

	// camera
	cv::Mat camera_matrix;
	J_COMMON.config_["camera_matrix"] 			>> camera_matrix;
	cv::Mat distortion_coefficients;
	J_COMMON.config_["distortion_coefficients"] >> distortion_coefficients;

	// laser module param
	cv::FileNode laser_param = J_COMMON.config_["laser_param"];
	Detector::LaserParam laserparam;
	laser_param["max_error"]           >> laserparam.max_error;
	laser_param["min_ratio"] 		   >> laserparam.min_Ratio;

	// filter param
	cv::FileNode filter_param = J_COMMON.config_["filter_param"];
	AdaptiveLowPassFilter::FilterParam fp;
	filter_param["alpha_static"]  	   >> fp.alpha_static;
	filter_param["alpha_dynamic"]	   >> fp.alpha_dynamic;
	filter_param["motion_thresh"]	   >> fp.motion_thresh;
	filter_param["smooth_rate"] 	   >> fp.smooth_rate;

	float confidence, binary_thresh, detector_color;
	J_COMMON.config_["confidence"] 	   >> confidence;
	J_COMMON.config_["binary_thresh"]  >> binary_thresh;
	J_COMMON.config_["detector_color"] >> detector_color;
	YOLO yolo("models/classes.txt", "models/module.onnx", confidence);
	Detector detector(binary_thresh, detector_color, laserparam);
	AdaptiveLowPassFilter ALPfilter(fp);
	PnPSolver pnpsolver(camera_matrix,distortion_coefficients);
	UI ui;

   	float cam2gimDis,gim2odom_angle,gim2odomDis,gimbal_pitch,gimbal_yaw,roll;
    cv::Mat odom_tvec;
    cv::Mat odom_rvec;
	cv::FileNode transform_param = J_COMMON.config_["transform_param"];
	transform_param["cam2gimDis"]	   >> cam2gimDis;
	transform_param["gim2odom_angle"]  >> gim2odom_angle;
	transform_param["gim2odomDis"]	   >> gim2odomDis;
	transform_param["gimbal_pitch"]	   >> gimbal_pitch;
	transform_param["gimbal_yaw"] 	   >> gimbal_yaw;
	transform_param["roll"] 	       >> roll;

	// 相机
	// HikCamera camera(0);

	// test video
	// cv::VideoCapture camera("docs/Video_20260315182119088.avi");
	cv::VideoCapture camera("docs/Video_20260315182913969.avi");
	// cv::VideoCapture camera("docs/60s.mp4");

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
		pnpsolver.solvePnP(laserModule,rvec,tvec);
		pnpsolver.calculateDistanceToCenter(laserModule.center);
		TransForm::coordinateTransform(tvec, rvec, cam2gimDis,gim2odom_angle,gim2odomDis,gimbal_pitch,gimbal_yaw,odom_tvec,odom_rvec,roll);
		
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