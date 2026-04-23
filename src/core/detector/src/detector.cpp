#include "detector/detector.hpp"

Detector::Detector(
    const int& bin_thresh, const int& color, const LaserParam& laserparam)
    : binary_thresh(bin_thresh), detect_color(color), lp(laserparam)
{
}

LaserModule Detector::lasermoduleDetector(const cv::Mat& frame, std::vector<Detection> results)
{
    Laser_Areas.clear();
    Laser_Areas = extractLaser(frame, results);
    for (auto& Laser_Area : Laser_Areas)
    {
        binary_img = preprocessImage(Laser_Area);
        Rects = lightExtractor(binary_img);
        lm.rects = Rects;
        min_Rect = laserArea(binary_img, Rects);
        lm.rect = min_Rect;
        lm.Centers = getCenter(min_Rect);
    }
    return lm;
}

std::vector<cv::Mat> Detector::extractLaser(const cv::Mat& frame, std::vector<Detection>& results)
{
    std::vector<cv::Mat> laser_areas;
    laser_areas.reserve(results.size());

    cv::Rect img_rect(0, 0, frame.cols, frame.rows);

    for (const auto& result : results)
    {
        cv::Rect box = result.box & img_rect;
        x_ = result.box.x;
        y_ = result.box.y;
        if (box.area() <= 0)
            continue;
        laser_areas.push_back(frame(box));
    }
    
    return laser_areas;
}

cv::Mat Detector::preprocessImage(const cv::Mat& rgb_img)
{
    std::vector<cv::Mat> channels;
    cv::split(rgb_img, channels);
    cv::Mat red = channels[2];
    cv::Mat blue = channels[0];

    cv::Mat binary_img;
    cv::threshold((detect_color == RED ? red : blue), binary_img, binary_thresh, 255, cv::THRESH_BINARY);

    //  X 方向累积投影
    std::vector<float> projX(binary_img.cols, 0.f);
    for (int r = 0; r < binary_img.rows; ++r) {
        const uchar* p = binary_img.ptr<uchar>(r);
        for (int c = 0; c < binary_img.cols; ++c)
            if (p[c]) projX[c] += 1.f;
    }

    float maxVal = *std::max_element(projX.begin(), projX.end());
    float thresh = maxVal * 0.3f; // 低于峰值30%视为谷

    std::vector<std::pair<int,int>> peaks;
    int peakStart = -1;
    for (int c = 0; c < (int)projX.size(); ++c) {
        if (projX[c] > thresh && peakStart < 0) peakStart = c;
        if (projX[c] <= thresh && peakStart >= 0) {
            peaks.push_back({peakStart, c});
            peakStart = -1;
        }
    }
    if (peakStart >= 0) peaks.push_back({peakStart, binary_img.cols});

    // 切割像素宽度
    const int cutWidth = 5;
    for (int i = 1; i < (int)peaks.size(); ++i) {
        int lo = peaks[i-1].second;
        int hi = peaks[i].first;
        int minIdx = lo;
        for (int c = lo + 1; c < hi; ++c)
            if (projX[c] < projX[minIdx]) minIdx = c;
        cv::rectangle(binary_img,
            cv::Point(minIdx - cutWidth / 2, 0),
            cv::Point(minIdx + cutWidth / 2, binary_img.rows - 1),
            0, cv::FILLED);
    }
    cv::imshow("binary_img", binary_img);
    return binary_img;
}

cv::Rect Detector::laserArea(const cv::Mat& binary_image, const std::vector<cv::Rect>& rects)
{
    if (rects.empty()) return cv::Rect();
    size_t n = rects.size();
    // std::cout << n << std::endl;
    if (n != 4 && n != 6) 
    {
        lm.type = LaserType::INVALID;
        return cv::Rect();
    }
    lm.type = (n == 4) ? LaserType::FOUR : LaserType::SIX;

    std::vector<cv::Point> points;
    for (const auto& r : rects) {
        points.push_back(r.tl());
        points.push_back(r.br());
    }

    cv::Rect roi = cv::boundingRect(points);
    roi.x += x_;
    roi.y += y_;

    return roi;
}

std::vector<cv::Point2f> Detector::getCenter(cv::Rect& min_rect)
{
    std::vector<cv::Point2f> points;
    lm.rect = min_rect;
    lm.center = cv::Point2f(min_rect.x + min_rect.width/2.0f, min_rect.y + min_rect.height/2.0f);;
    
    float width = min_rect.width;
    float height = min_rect.height;
    if (width == 0 || height == 0) return points;
    // std::cout << "Laser Ratio: " << width / height << std::endl;

    // 6灯
    if (lm.type == LaserType::SIX)
    {
        // 排序顺序：从左上开始逆时针排序
        points = 
        {
            cv::Point2f(width * 0.1 + min_rect.x, height * 0.08 + min_rect.y),
            cv::Point2f(width * 0.5 + min_rect.x, height * 0.08 + min_rect.y),
            cv::Point2f(width * 0.9 + min_rect.x, height * 0.08 + min_rect.y),
            cv::Point2f(width * 0.9 + min_rect.x, height * 0.92 + min_rect.y),
            cv::Point2f(width * 0.5 + min_rect.x, height * 0.92 + min_rect.y),
            cv::Point2f(width * 0.1 + min_rect.x, height * 0.92 + min_rect.y)
        };
    }
    // 4灯
    else
    {
        points = 
        {
            cv::Point2f(width * 0.148148 + min_rect.x, height * 0.08 + min_rect.y),
            cv::Point2f(width * 0.740740 + min_rect.x, height * 0.08 + min_rect.y),
            cv::Point2f(width * 0.740740 + min_rect.x, height * 0.92 + min_rect.y),
            cv::Point2f(width * 0.148148 + min_rect.x, height * 0.92 + min_rect.y)
        };
    }
    return points;
}

std::vector<cv::Rect> Detector::lightExtractor(const cv::Mat& binary_img)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Rect> rects;
    std::vector<cv::Rect> rects_;

    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours)
    {
        auto r_rect = cv::minAreaRect(contour);
        auto rect = r_rect.boundingRect();
        if (r_rect.size.width <= 0 || r_rect.size.height <= 0) continue;
        float ratio = r_rect.size.width / r_rect.size.height;
        // std::cout << "Light Area: " << rect.area() << std::endl;
        // if (!isLightArea(rect,ratio)) continue;
        // std::cout << "Light Ratio: " << ratio << std::endl;
        rects.push_back(rect);
    }

    if (rects.empty()) return rects;
    // std::cout << "Light Number: " << rects.size() << std::endl;
    std::set<size_t> added_indices;
    for (size_t i = 0; i < rects.size(); ++i)
    {
        for (size_t j = i + 1; j < rects.size(); ++j)
        {
            if (isLight(rects[i], rects[j]))
            {
                added_indices.insert(i);
                added_indices.insert(j);
            }
        }
    }
    for (size_t idx : added_indices)
    {
        rects_.push_back(rects[idx]);
    }

    // std::cout << "Light Number(过滤后): " << rects_.size() << std::endl;

    return rects_;
}

bool Detector::isLightArea(const cv::Rect& rect, const float& ratio)
{
    bool area_ok =  rect.area() < 10;
    bool ratio_ok = ratio < lp.min_Ratio;

    bool all_ok = area_ok && ratio_ok;
    return all_ok;
}

bool Detector::isLight(const cv::Rect& rect1, const cv::Rect& rect2)
{
    bool x_similar = cv::abs(rect1.x -rect2.x) < lp.max_error;
    bool y_NSimilar = cv::abs(rect1.y -rect2.y) > lp.max_error;
    float area_ratio = rect1.area() / rect2.area();
    // bool area_ratio_ok = (area_ratio > 0.5 || area_ratio < 2); // 面积比例参数
    // std::cout << "x坐标差值: " << cv::abs(rect1.center.x -rect2.center.x) << std::endl;
    // std::cout << "y坐标差值: " << cv::abs(rect1.center.y -rect2.center.y) << std::endl;

    bool all_ok = x_similar && y_NSimilar;

    return all_ok;
}