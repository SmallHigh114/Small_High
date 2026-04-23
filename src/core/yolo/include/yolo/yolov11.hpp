#ifndef YOLOV11_HPP
#define YOLOV11_HPP

#include <openvino/openvino.hpp>    
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @brief 单个目标检测结果结构体
 */
struct Detection {
    int class_id;          // 类别ID
    float confidence;      // 置信度
    cv::Rect box;          // 目标框
    std::string label;     // 类别名称
};

/**
 * @brief YOLOv11 目标检测封装类
 */
class YOLO {
public:

    /**
     * @brief 构造函数
     *
     * @param labels_txt_file 类别文件路径
     * @param modelPath ONNX模型路径
     * @param conf_threshold 置信度阈值
     * @param nms_threshold NMS 阈值
     */
    YOLO(const std::string& labels_txt_file,
        const std::string& modelPath,
        float conf_threshold = 0.25f,
        float nms_threshold = 0.45f);

    /**
     * @brief 执行目标检测
     *
     * @param image 输入图像
     * 
     * @return 检测结果列表
     */
    std::vector<Detection> detect(const cv::Mat& image);

private:

    /**
     * @brief 加载类别名称
     *
     * @param path 类别文件路径
     * 
     * @return 类别字符串列表
     */
    std::vector<std::string> loadClassNames(const std::string& path);

    /**
     * @brief 等比例缩放并填充（Letterbox）
     *
     * @param src 输入图像
     * @param pad_x 水平填充
     * @param pad_y 垂直填充
     * @param scale 缩放比例
     * 
     * @return 处理后的图像
     */
    cv::Mat letterbox(const cv::Mat& src,
        int& pad_x,
        int& pad_y,
        float& scale);

    float IoU(const cv::Rect& a, const cv::Rect& b);

    void myNMSBoxes(
        const std::vector<cv::Rect>& boxes,
        const std::vector<float>& scores,
        float score_threshold,
        float nms_threshold,
        std::vector<int>& indices);

private:
    std::vector<std::string> labels_;  // 类别名称列表
    int num_classes_;                  // 类别数量
    float conf_threshold_;             // 置信度阈值
    float nms_threshold_;              // NMS阈值

    ov::Core core_;
    ov::CompiledModel compiled_model_; // 已编译模型
    ov::InferRequest request_;         // 推理请求

    size_t input_h_;                   // 模型输入高度
    size_t input_w_;                   // 模型输入宽度
    size_t ch_;                        // 输入通道数

    size_t out_channels_;              // 输出通道数
    size_t num_anchors_;               // anchor数量
};
#endif // YOLOV11_HPP