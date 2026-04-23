#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

#include <hik/MvCameraControl.h>

class HikCamera
{
public:

    HikCamera(int deviceIndex);
    ~HikCamera();

    HikCamera(const HikCamera&) = delete;
    HikCamera& operator= (const HikCamera&) = delete;

    void close();
    void setExposureTime(float exposureTime);
    void setGain(float gain);
    bool isOpened() const;


    HikCamera& operator>> (cv::Mat& image);

private:
    //init camera:
    void* m_hCamera = nullptr;
    MV_CC_DEVICE_INFO_LIST      m_deviceList;
    std::vector<unsigned char>  m_imageBuffer;
    //param:
    float                       p_exposureTime = 5000;
    float                       p_gain = 6.0f;
};