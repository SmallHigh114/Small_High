#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

#include "hikcamera/hikcamera.h"

/**
 * @brief Construct a new Hik Camera:: Hik Camera object
 *
 * @param deviceIndex
 */

HikCamera::HikCamera(int deviceIndex)
{
    memset(&this->m_deviceList, 0, sizeof(this->m_deviceList));

    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &this->m_deviceList);

    MVCC_INTVALUE width, height;

    if (nRet != MV_OK || this->m_deviceList.nDeviceNum <= 0) {

        std::cerr << "[HikCamera]: NO Camera Found!" << std::endl;
        this->m_hCamera = nullptr;
        return;
    }

    if (static_cast<unsigned int>(deviceIndex) >= this->m_deviceList.nDeviceNum) {
        deviceIndex = 0;
    }

    nRet = MV_CC_CreateHandle(&this->m_hCamera, this->m_deviceList.pDeviceInfo[deviceIndex]);
    if (nRet != MV_OK) {
        std::cerr << "[HikCamera]: Create Handle Failed!" << std::endl;
    }

    nRet = MV_CC_OpenDevice(this->m_hCamera);
    if (nRet != MV_OK) {
        std::cerr << "[HikCamera]: Open Device Failed!" << std::endl;
    }
    MV_CC_SetIntValue(m_hCamera, "OffsetX", 0);
    MV_CC_SetIntValue(m_hCamera, "OffsetY", 0);

    MV_CC_SetIntValue(m_hCamera, "Width", 960);
    MV_CC_SetIntValue(m_hCamera, "Height", 720);

    MV_CC_SetBayerCvtQuality(this->m_hCamera, 1);
    MV_CC_SetPixelFormat(this->m_hCamera, PixelType_Gvsp_BGR8_Packed);

    MV_CC_SetEnumValue(m_hCamera, "BalanceWhiteAuto", 1);
    MV_CC_SetFloatValue(m_hCamera, "ExposureTime", this->p_exposureTime);
    MV_CC_SetFloatValue(m_hCamera, "Gain", this->p_gain);

    MV_CC_GetIntValue(m_hCamera, "Width", &width);
    MV_CC_GetIntValue(m_hCamera, "Height", &height);

    m_imageBuffer.resize(width.nCurValue * height.nCurValue * 3);

    MV_CC_StartGrabbing(m_hCamera);
}

/**
 * @brief Destroy the Hik Camera:: Hik Camera object
 *
 */
HikCamera::~HikCamera()
{
    this->close();
}

/**
 * @brief determine the opening status of camera
 * @details if the camera opening correctly,
 *          the handle of camera must not be empty
 * @return true
 * @return false
 */
bool HikCamera::isOpened() const
{
    return this->m_hCamera != nullptr;
}

/**
 * @brief helping destructor, release camera handle
 *
 */
void HikCamera::close()
{
    if (this->m_hCamera) {

        MV_CC_StopGrabbing(this->m_hCamera);
        MV_CC_CloseDevice(this->m_hCamera);
        MV_CC_DestroyHandle(this->m_hCamera);

        this->m_hCamera = nullptr;
    }
}

/**
 * @brief the interface of setting expourse time
 *
 * @param exposureTime
 */
void HikCamera::setExposureTime(float exposureTime) {
    this->p_exposureTime = exposureTime;
    MV_CC_SetFloatValue(this->m_hCamera, "ExposureTime", exposureTime);
}

void HikCamera::setGain(float gain) {
    this->p_gain = gain;
    MV_CC_SetFloatValue(this->m_hCamera, "Gain", gain);
}

/**
 * @brief overload the operator ">>" ,makes call more convenient
 *          just like "cv::VideoCapture"
 *
 * @param outImage
 * @return HikCamera&
 */
HikCamera& HikCamera::operator>> (cv::Mat& outImage)
{
    if (!m_hCamera) {
        outImage.release();
        return *this;
    }

    MV_FRAME_OUT_INFO_EX frameInfo = { 0 };

    int nRet = MV_CC_GetOneFrameTimeout(this->m_hCamera, m_imageBuffer.data(), static_cast<unsigned int>(this->m_imageBuffer.size()), &frameInfo, 1000);

    if (nRet != MV_OK) {
        outImage.release();
        return *this;
    }

    outImage = cv::Mat(frameInfo.nHeight, frameInfo.nWidth, CV_8UC3, this->m_imageBuffer.data());

    return *this;
}