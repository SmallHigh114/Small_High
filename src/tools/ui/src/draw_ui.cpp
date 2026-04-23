#include "ui/draw_ui.hpp"
#include <iomanip>

void UI::drawResult(cv::Mat& frame, LaserModule laserModule)
{
        cv::rectangle(frame, laserModule.rect, cv::Scalar(0, 255, 0), 2);
        
        if (laserModule.Centers.empty()) return;
        cv::RotatedRect rect = cv::minAreaRect(laserModule.Centers);

        cv::Point2f corners[4];
        rect.points(corners);

        for (int i = 0; i < 4; i++)
        {
            cv::line(frame, corners[i], corners[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
        }
}

void UI::drawFps(cv::Mat& frame)
{
    static auto last = std::chrono::steady_clock::now();
    auto now  = std::chrono::steady_clock::now();
    float fps = 1.0f / std::chrono::duration<float>(now - last).count();
    last = now;

    char buf[32];
    std::snprintf(buf, sizeof(buf), "FPS: %.1f", fps);
    cv::putText(frame, buf, {10, 30},
                cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
}

void UI::drawDetections(cv::Mat& image, const std::vector<Detection>& detections)
{
    for (const auto& det : detections)
    {
        if (det.box.width <= 0 || det.box.height <= 0)
            continue;

        cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2);

        std::string label = det.label.empty() ? "unknown" : det.label;
        std::string text = label + ": " + cv::format("%.2f", det.confidence);

        int baseline = 0;
        cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);

        int y = det.box.y > ts.height ? det.box.y - 4 : det.box.y + ts.height;
        cv::Point org(det.box.x, y);

        cv::rectangle(image,
            cv::Point(org.x, org.y - ts.height - baseline),
            cv::Point(org.x + ts.width, org.y + baseline),
            cv::Scalar(0, 255, 0), cv::FILLED);

        cv::putText(image, text, org,
            cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

        std::cout << "检测到: " << label
            << "  置信度: " << det.confidence
            << "  位置: [" << det.box.x << "," << det.box.y
            << "," << det.box.width << "," << det.box.height << "]\n";
    }
}

void UI::pushRaw(RawHistory& raw, float x, float y)
{
    raw.x.push_back(x);
    raw.y.push_back(y);
    // 原始历史只需保留 SMOOTH_WIN 帧（够算均值即可）
    // 但为了支持 HISTORY_LEN 长度的抖动队列，保留 HISTORY_LEN 帧
    if ((int)raw.x.size() > HISTORY_LEN) { raw.x.pop_front(); raw.y.pop_front(); }
}

void UI::pushJitter(JitterHistory& hist, float dx, float dy)
{
    hist.dx.push_back(dx);
    hist.dy.push_back(dy);
    if ((int)hist.dx.size() > HISTORY_LEN) { hist.dx.pop_front(); hist.dy.pop_front(); }
}

void UI::computeJitter(const RawHistory& raw, float& out_dx, float& out_dy)
{
    int n = (int)raw.x.size();
    if (n == 0) { out_dx = 0.f; out_dy = 0.f; return; }

    float cur_x = raw.x.back();
    float cur_y = raw.y.back();

    // 用最近 SMOOTH_WIN 帧（不含当前）计算参考均值
    int win_start = std::max(0, n - 1 - SMOOTH_WIN); // 不含最后一帧（当前帧）
    int win_end   = n - 1;                            // 不含（exclusive）
    int win_len   = win_end - win_start;

    if (win_len <= 0)
    {
        // 只有 1 帧，无参考，抖动为 0
        out_dx = 0.f;
        out_dy = 0.f;
        return;
    }

    float sum_x = 0.f, sum_y = 0.f;
    for (int i = win_start; i < win_end; ++i)
    {
        sum_x += raw.x[i];
        sum_y += raw.y[i];
    }
    float mean_x = sum_x / win_len;
    float mean_y = sum_y / win_len;

    out_dx = cur_x - mean_x;
    out_dy = cur_y - mean_y;
}

void UI::drawSingleWaveform(
    cv::Mat& canvas,
    const std::deque<float>& dxData,
    const std::deque<float>& dyData,
    const std::string& title,
    const cv::Scalar& colorDx,
    const cv::Scalar& colorDy)
{
    const int W         = canvas.cols;
    const int H         = canvas.rows;
    const int PAD_LEFT  = 52;
    const int PAD_RIGHT = 12;
    const int PAD_TOP   = 26;
    const int PAD_BOT   = 18;
    const int plotW     = W - PAD_LEFT - PAD_RIGHT;
    const int plotH     = H - PAD_TOP  - PAD_BOT;

    canvas.setTo(cv::Scalar(18, 18, 18));

    // 绘图区边框
    cv::rectangle(canvas,
        cv::Point(PAD_LEFT, PAD_TOP),
        cv::Point(PAD_LEFT + plotW, PAD_TOP + plotH),
        cv::Scalar(70, 70, 70), 1);

    // 标题
    cv::putText(canvas, title, cv::Point(PAD_LEFT, PAD_TOP - 6),
        cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);

    // 图例
    const int legX = PAD_LEFT + plotW - 110;
    cv::line(canvas, cv::Point(legX, PAD_TOP + 10), cv::Point(legX + 16, PAD_TOP + 10), colorDx, 2);
    cv::putText(canvas, "dX", cv::Point(legX + 20, PAD_TOP + 14),
        cv::FONT_HERSHEY_SIMPLEX, 0.36, colorDx, 1, cv::LINE_AA);
    cv::line(canvas, cv::Point(legX + 42, PAD_TOP + 10), cv::Point(legX + 58, PAD_TOP + 10), colorDy, 2);
    cv::putText(canvas, "dY", cv::Point(legX + 62, PAD_TOP + 14),
        cv::FONT_HERSHEY_SIMPLEX, 0.36, colorDy, 1, cv::LINE_AA);

    if (dxData.empty()) return;

    // 自适应 Y 轴量程（两条曲线共用）
    float vmin =  std::numeric_limits<float>::max();
    float vmax = -std::numeric_limits<float>::max();
    for (float v : dxData) { vmin = std::min(vmin, v); vmax = std::max(vmax, v); }
    for (float v : dyData) { vmin = std::min(vmin, v); vmax = std::max(vmax, v); }

    float range = vmax - vmin;
    if (range < 0.5f) range = 2.0f;  // 抖动极小时给一个最小显示量程（±1px）
    float vpad = range * 0.15f;
    float lo   = vmin - vpad;
    float hi   = vmax + vpad;
    float span = hi - lo;

    // 零线（抖动参考基准）
    {
        int zeroY = PAD_TOP + plotH - (int)((0.f - lo) / span * plotH);
        zeroY = std::max(PAD_TOP, std::min(PAD_TOP + plotH, zeroY));
        cv::line(canvas,
            cv::Point(PAD_LEFT, zeroY),
            cv::Point(PAD_LEFT + plotW, zeroY),
            cv::Scalar(55, 55, 55), 1);
        cv::putText(canvas, "0", cv::Point(2, zeroY + 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
    }

    // Y 轴刻度（上下边界）
    for (int k = 0; k <= 2; ++k)
    {
        float ratio = k / 2.0f;
        int   gy    = PAD_TOP + plotH - (int)(ratio * plotH);
        float gval  = lo + ratio * span;

        cv::line(canvas,
            cv::Point(PAD_LEFT, gy),
            cv::Point(PAD_LEFT + plotW, gy),
            cv::Scalar(38, 38, 38), 1);

        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.1f", gval);
        cv::putText(canvas, buf, cv::Point(2, gy + 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.28, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);
    }

    auto toPixY = [&](float v) -> int {
        int py = PAD_TOP + plotH - (int)(((v - lo) / span) * plotH);
        return std::max(PAD_TOP, std::min(PAD_TOP + plotH, py));
    };

    int n = (int)dxData.size();

    // 绘制 dX 折线
    for (int i = 1; i < n; ++i)
    {
        int x0 = PAD_LEFT + (int)(((i - 1) / (float)(HISTORY_LEN - 1)) * plotW);
        int x1 = PAD_LEFT + (int)((i       / (float)(HISTORY_LEN - 1)) * plotW);
        cv::line(canvas,
            cv::Point(x0, toPixY(dxData[i - 1])),
            cv::Point(x1, toPixY(dxData[i])),
            colorDx, 1, cv::LINE_AA);
    }

    // 绘制 dY 折线
    for (int i = 1; i < n; ++i)
    {
        int x0 = PAD_LEFT + (int)(((i - 1) / (float)(HISTORY_LEN - 1)) * plotW);
        int x1 = PAD_LEFT + (int)((i       / (float)(HISTORY_LEN - 1)) * plotW);
        cv::line(canvas,
            cv::Point(x0, toPixY(dyData[i - 1])),
            cv::Point(x1, toPixY(dyData[i])),
            colorDy, 1, cv::LINE_AA);
    }

    // 最新值端点标注
    if (n > 0)
    {
        int lastPx = PAD_LEFT + (int)(((n - 1) / (float)(HISTORY_LEN - 1)) * plotW);

        char bufDx[16], bufDy[16];
        std::snprintf(bufDx, sizeof(bufDx), "%.2f", dxData.back());
        std::snprintf(bufDy, sizeof(bufDy), "%.2f", dyData.back());

        int pyDx = toPixY(dxData.back());
        int pyDy = toPixY(dyData.back());

        cv::circle(canvas, cv::Point(lastPx, pyDx), 3, colorDx, cv::FILLED);
        cv::circle(canvas, cv::Point(lastPx, pyDy), 3, colorDy, cv::FILLED);

        int textX = std::min(lastPx + 4, W - 36);
        cv::putText(canvas, bufDx, cv::Point(textX, pyDx + 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.30, colorDx, 1, cv::LINE_AA);
        cv::putText(canvas, bufDy, cv::Point(textX, pyDy + 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.30, colorDy, 1, cv::LINE_AA);
    }
}

void UI::drawWaveforms(const cv::Rect& minRect, const LaserModule& laserModule)
{
    // ---- 1. 提取当前帧中心坐标 ----

    // 绿色框（min_Rect）中心
    float gx = minRect.x + minRect.width  / 2.0f;
    float gy = minRect.y + minRect.height / 2.0f;

    // 蓝色框（Centers 的 minAreaRect）中心
    float bx = 0.f, by = 0.f;
    if (!laserModule.Centers.empty())
    {
        cv::RotatedRect rr = cv::minAreaRect(laserModule.Centers);
        bx = rr.center.x;
        by = rr.center.y;
    }

    // ---- 2. 推入原始历史 ----
    pushRaw(raw_green_, gx, gy);
    pushRaw(raw_blue_,  bx, by);

    // ---- 3. 计算相对抖动偏差 ----
    float g_dx, g_dy, b_dx, b_dy;
    computeJitter(raw_green_, g_dx, g_dy);
    computeJitter(raw_blue_,  b_dx, b_dy);

    // 相互抖动偏差：两框抖动之差，反映两者相对运动
    float r_dx = g_dx - b_dx;
    float r_dy = g_dy - b_dy;

    // ---- 4. 推入抖动历史 ----
    pushJitter(jitter_green_,    g_dx, g_dy);
    pushJitter(jitter_blue_,     b_dx, b_dy);
    pushJitter(jitter_relative_, r_dx, r_dy);

    // ---- 5. 控制台日志 ----
    // 每帧输出一行，格式对齐，单位：像素
    std::cout << std::fixed << std::setprecision(2)
              << "[Jitter] "
              << "Green(dX=" << std::setw(7) << g_dx << " dY=" << std::setw(7) << g_dy << " px) | "
              << "Blue (dX=" << std::setw(7) << b_dx << " dY=" << std::setw(7) << b_dy << " px) | "
              << "Relative(dX=" << std::setw(7) << r_dx << " dY=" << std::setw(7) << r_dy << " px)"
              << "\n";

    // ---- 6. 构建合并画布（3 行子图） ----
    const int WIN_W  = 720;
    const int PLOT_H = 130;
    const int SEP    = 6;
    const int WIN_H  = PLOT_H * 3 + SEP * 4;

    cv::Mat waveCanvas(WIN_H, WIN_W, CV_8UC3, cv::Scalar(10, 10, 10));

    // 子图1：绿色框相对抖动
    {
        cv::Rect roi(0, SEP, WIN_W, PLOT_H);
        cv::Mat sub = waveCanvas(roi);
        drawSingleWaveform(sub,
            jitter_green_.dx, jitter_green_.dy,
            "Waveform 1 | Green Box (min_Rect) Relative Jitter [px]",
            cv::Scalar(100, 230, 100),   // dX：浅绿
            cv::Scalar(0,   180,  60));  // dY：深绿
    }

    // 子图2：蓝色框相对抖动
    {
        cv::Rect roi(0, SEP * 2 + PLOT_H, WIN_W, PLOT_H);
        cv::Mat sub = waveCanvas(roi);
        drawSingleWaveform(sub,
            jitter_blue_.dx, jitter_blue_.dy,
            "Waveform 2 | Blue Box (Centers minAreaRect) Relative Jitter [px]",
            cv::Scalar(100, 180, 255),   // dX：浅蓝
            cv::Scalar(50,   80, 220));  // dY：深蓝
    }

    // 子图3：两框相互抖动偏差
    {
        cv::Rect roi(0, SEP * 3 + PLOT_H * 2, WIN_W, PLOT_H);
        cv::Mat sub = waveCanvas(roi);
        drawSingleWaveform(sub,
            jitter_relative_.dx, jitter_relative_.dy,
            "Waveform 3 | Green-Blue Relative Jitter Difference [px]",
            cv::Scalar(255, 180,  60),   // dX：橙
            cv::Scalar(200,  80, 200));  // dY：紫
    }

    // 子图间分隔线
    for (int k = 1; k <= 2; ++k)
    {
        int lineY = SEP + PLOT_H * k + SEP * k - SEP / 2;
        cv::line(waveCanvas,
            cv::Point(0, lineY),
            cv::Point(WIN_W, lineY),
            cv::Scalar(45, 45, 45), 1);
    }

    cv::imshow("Jitter Waveforms", waveCanvas);
}