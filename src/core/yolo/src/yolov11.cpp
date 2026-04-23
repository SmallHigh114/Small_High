#include "yolo/yolov11.hpp"
#include <fstream>
#include <iostream>

YOLO::YOLO(const std::string& labels_txt_file,
    const std::string& modelPath,
    float conf_threshold,
    float nms_threshold)
    : conf_threshold_(conf_threshold), nms_threshold_(nms_threshold)
{
    // 加载标签
    labels_ = loadClassNames(labels_txt_file);
    num_classes_ = static_cast<int>(labels_.size());

    // 加载并编译模型
    compiled_model_ = core_.compile_model(modelPath, "AUTO");
    request_ = compiled_model_.create_infer_request();

    // 缓存输入形状
    ov::Shape input_shape = request_.get_input_tensor(0).get_shape();
    ch_ = input_shape[1];
    input_h_ = input_shape[2];
    input_w_ = input_shape[3];

    // 缓存输出形状
    ov::Shape output_shape = request_.get_output_tensor(0).get_shape();
    out_channels_ = output_shape[1]; // 4 + num_classes
    num_anchors_ = output_shape[2];
#ifdef YOLO_DEBUG
    std::cout << "[YOLO] Model loaded. Input: "
        << input_shape[0] << "x" << ch_ << "x" << input_h_ << "x" << input_w_
        << "  Output: 1x" << out_channels_ << "x" << num_anchors_ << std::endl;
#endif // YOLO_DEBUG
}

std::vector<Detection> YOLO::detect(const cv::Mat& image)
{
    int   img_h = image.rows;
    int   img_w = image.cols;
    int   pad_x, pad_y;
    float scale;

    // 预处理
    cv::Mat padded = letterbox(image, pad_x, pad_y, scale);

    cv::Mat blob;
    padded.convertTo(blob, CV_32F, 1.0 / 255.0);

    // 填充 input tensor（HWC => NCHW）
    ov::Tensor input_tensor = request_.get_input_tensor(0);
    float* data = input_tensor.data<float>();
    int    image_size = static_cast<int>(input_w_ * input_h_);

    for (size_t row = 0; row < input_h_; row++)
        for (size_t col = 0; col < input_w_; col++)
            for (size_t c = 0; c < ch_; c++)
                data[image_size * c + row * input_w_ + col] = blob.at<cv::Vec3f>(row, col)[c];

    // 推理
    request_.infer();

    // 后处理
    float* out_data = request_.get_output_tensor(0).data<float>();

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;
    std::vector<int>      class_ids;

    for (size_t i = 0; i < num_anchors_; i++) {
        float cx = out_data[0 * num_anchors_ + i];
        float cy = out_data[1 * num_anchors_ + i];
        float bw = out_data[2 * num_anchors_ + i];
        float bh = out_data[3 * num_anchors_ + i];

        float max_score = -1.0f;
        int   max_id = -1;
        for (int c = 0; c < num_classes_; c++) {
            float score = out_data[(4 + c) * num_anchors_ + i];
            if (score > max_score) { max_score = score; max_id = c; }
        }
        if (max_score < conf_threshold_) continue;

        // 还原到原图坐标
        float x1 = (cx - bw / 2.0f - pad_x) / scale;
        float y1 = (cy - bh / 2.0f - pad_y) / scale;
        float x2 = (cx + bw / 2.0f - pad_x) / scale;
        float y2 = (cy + bh / 2.0f - pad_y) / scale;

        x1 = std::max(0.0f, std::min(x1, (float)(img_w - 1)));
        y1 = std::max(0.0f, std::min(y1, (float)(img_h - 1)));
        x2 = std::max(0.0f, std::min(x2, (float)(img_w - 1)));
        y2 = std::max(0.0f, std::min(y2, (float)(img_h - 1)));

        boxes.push_back(cv::Rect((int)x1, (int)y1, (int)(x2 - x1), (int)(y2 - y1)));
        confidences.push_back(max_score);
        class_ids.push_back(max_id);
    }

    // NMS
    std::vector<int> indices;
    myNMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    // 组装结果
    std::vector<Detection> results;
    results.reserve(indices.size());
    for (int idx : indices) {
        Detection det;
        det.class_id = class_ids[idx];
        det.confidence = confidences[idx];
        det.box = boxes[idx];
        det.label = labels_[det.class_id];
        results.push_back(det);
    }
    return results;
}

std::vector<std::string> YOLO::loadClassNames(const std::string& path)
{
    std::vector<std::string> names;
    std::ifstream fp(path);
    if (!fp.is_open()) { std::cerr << "无法打开标签文件: " << path << std::endl; exit(-1); }
    std::string line;
    while (std::getline(fp, line))
        if (!line.empty()) names.push_back(line);
    return names;
}

cv::Mat YOLO::letterbox(const cv::Mat& src, int& pad_x, int& pad_y, float& scale)
{
    int img_h = src.rows, img_w = src.cols;
    scale = std::min((float)input_w_ / img_w, (float)input_h_ / img_h);
    int new_w = (int)(img_w * scale);
    int new_h = (int)(img_h * scale);
    pad_x = ((int)input_w_ - new_w) / 2;
    pad_y = ((int)input_h_ - new_h) / 2;

    cv::Mat rgb, resized;
    cv::cvtColor(src, rgb, cv::COLOR_BGR2RGB);
    cv::resize(rgb, resized, cv::Size(new_w, new_h));

    cv::Mat padded((int)input_h_, (int)input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return padded;
}

float YOLO::IoU(const cv::Rect& a, const cv::Rect& b)
{
    int inter_x1 = std::max(a.x, b.x);
    int inter_y1 = std::max(a.y, b.y);
    int inter_x2 = std::min(a.x + a.width, b.x + b.width);
    int inter_y2 = std::min(a.y + a.height, b.y + b.height);

    int inter_w = std::max(0, inter_x2 - inter_x1);
    int inter_h = std::max(0, inter_y2 - inter_y1);

    float inter_area = inter_w * inter_h;
    float union_area = a.area() + b.area() - inter_area;

    return inter_area / union_area;
}

void YOLO::myNMSBoxes(
    const std::vector<cv::Rect>& boxes,
    const std::vector<float>& scores,
    float score_threshold,
    float nms_threshold,
    std::vector<int>& indices)
{
    std::vector<int> order;

    // 过滤低置信度
    for (int i = 0; i < scores.size(); i++)
    {
        if (scores[i] >= score_threshold)
            order.push_back(i);
    }

    // 按置信度排序
    std::sort(order.begin(), order.end(),
        [&](int a, int b)
        {
            return scores[a] > scores[b];
        });

    std::vector<bool> suppressed(order.size(), false);

    // NMS
    for (int i = 0; i < order.size(); i++)
    {
        if (suppressed[i])
            continue;

        int idx = order[i];
        indices.push_back(idx);

        for (int j = i + 1; j < order.size(); j++)
        {
            if (suppressed[j])
                continue;

            int idx2 = order[j];

            float iou = IoU(boxes[idx], boxes[idx2]);

            if (iou > nms_threshold)
                suppressed[j] = true;
        }
    }
}