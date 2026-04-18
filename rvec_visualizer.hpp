#pragma once

/**
 * @file rvec_visualizer.hpp
 * @brief 旋转向量姿态箭头可视化（基于 OpenCV viz 模块）
 *
 * 可视化方案
 * ----------
 *   用一支 **单箭头** 表示目标的最终姿态：
 *     箭头方向 = 旋转后的 X 轴正方向（物体"朝前"方向）
 *     箭头根部 = 世界原点
 *     箭头颜色 = 亮橙色，醒目易辨
 *
 *   不拆解 XYZ 三轴，整体感更强。
 *
 * 坐标系约定（右手系）
 * --------------------
 *   X → 前（Forward）
 *   Y → 左（Left）
 *   Z → 上（Up）
 *
 * 初始视角
 * --------
 *   从 X 轴负半轴朝原点看，Z 轴朝上。
 *   等价于：站在目标"身后"面对目标，即"看自己右手"的视角。
 *
 * 用法
 * ----
 *   // 1. 阻塞
 *   visualizeRvec(rvec);
 *
 *   // 2. 实例化，循环更新
 *   RvecVisualizer vis;
 *   while (vis.isRunning()) {
 *       vis.update(rvec);
 *   }
 *
 * 依赖
 * ----
 *   OpenCV >= 3.x，编译时开启 viz 模块（需要 VTK）
 *   CMake: find_package(OpenCV REQUIRED COMPONENTS core viz)
 */

#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/types.hpp>
#include <string>
#include <stdexcept>
#include <cmath>

// ============================================================================
class RvecVisualizer
{
public:
    // ------------------------------------------------------------------------
    // 构造
    // ------------------------------------------------------------------------

    /**
     * @param window_name  窗口标题
     * @param arrow_length 箭头长度（米），同时也决定参考坐标系大小
     */
    explicit RvecVisualizer(const std::string& window_name = "Pose Arrow",
                            double             arrow_length = 0.25)
        : window_(window_name)
        , arrow_length_(arrow_length)
    {
        setupWindow();
    }

    ~RvecVisualizer() = default;

    // ------------------------------------------------------------------------
    // 公共接口
    // ------------------------------------------------------------------------

    /**
     * @brief 阻塞式展示。更新箭头后进入事件循环，关闭窗口后返回。
     * @param rvec  PnP 解算的旋转向量（cv::Vec3d）
     */
    void show(const cv::Vec3d& rvec)
    {
        update(rvec);
        window_.spin();
    }

    void show(const cv::Mat& rvec) { show(toVec3d(rvec)); }

    /**
     * @brief 非阻塞更新。刷新箭头姿态，内部调用 spinOnce。
     * @param rvec      旋转向量
     * @param delay_ms  spinOnce 等待时间（ms）
     */
    void update(const cv::Vec3d& rvec, int delay_ms = 1)
    {
        // rvec → 旋转矩阵
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // 旋转后的 X 轴方向（物体朝前方向）= R 的第 0 列
        cv::Vec3d xDir(
            R.at<double>(0, 0),
            R.at<double>(1, 0),
            R.at<double>(2, 0)
        );

        // 箭头：从原点指向 xDir * arrow_length_
        cv::Point3d tip(
            xDir[0] * arrow_length_,
            xDir[1] * arrow_length_,
            xDir[2] * arrow_length_
        );

        // WArrow(pt1=tail, pt2=tip, thickness, color)
        double thickness = arrow_length_ * 0.04;
        auto arrow = cv::viz::WArrow(
            cv::Point3d(0, 0, 0),
            tip,
            thickness,
            cv::viz::Color(255, 140, 0)   // 亮橙色
        );
        window_.showWidget("pose_arrow", arrow);

        // 旋转角度文字
        double angle_deg = cv::norm(rvec) * 180.0 / CV_PI;
        std::string info = "angle: " + std::to_string(static_cast<int>(std::round(angle_deg))) + " deg";
        auto txt = cv::viz::WText(info, cv::Point(10, 30), 16, cv::viz::Color::white());
        window_.showWidget("angle_text", txt);

        window_.spinOnce(delay_ms, true);
    }

    void update(const cv::Mat& rvec, int delay_ms = 1)
    {
        update(toVec3d(rvec), delay_ms);
    }

    void spin()                     { window_.spin(); }
    void spinOnce(int delay_ms = 1) { window_.spinOnce(delay_ms, true); }
    bool isRunning() const          { return !window_.wasStopped(); }

private:
    // ------------------------------------------------------------------------
    // 初始化
    // ------------------------------------------------------------------------

    cv::viz::Viz3d window_;
    double         arrow_length_;

    void setupWindow()
    {
        // 背景：深灰蓝
        window_.setBackgroundColor(cv::viz::Color(25, 25, 35));

        // ── 世界参考坐标系（小，仅作方向参考） ──────────────────────
        double refLen = arrow_length_ * 0.5;
        auto worldAxes = cv::viz::WCoordinateSystem(refLen);
        window_.showWidget("world_ref", worldAxes);

        // 三轴标签
        double lbl = refLen + arrow_length_ * 0.08;
        addLabel("lX", "X (fwd)",  cv::Point3d(lbl, 0,   0),   cv::viz::Color(100, 220, 100));
        addLabel("lY", "Y (left)", cv::Point3d(0,   lbl,  0),   cv::viz::Color(100, 160, 255));
        addLabel("lZ", "Z (up)",   cv::Point3d(0,   0,   lbl),  cv::viz::Color(255, 100, 100));

        // ── 占位箭头（初始指向 +X，update() 会覆盖） ──────────────
        double thickness = arrow_length_ * 0.04;
        auto initArrow = cv::viz::WArrow(
            cv::Point3d(0, 0, 0),
            cv::Point3d(arrow_length_, 0, 0),
            thickness,
            cv::viz::Color(255, 140, 0)
        );
        window_.showWidget("pose_arrow", initArrow);

        // ── 网格（XY 平面 Z=0，增强空间感） ────────────────────────
        addGrid();

        // ── 初始视角：从 X 负半轴看向原点，Z 朝上 ─────────────────
        double camDist = arrow_length_ * 4.5;
        window_.setViewerPose(
            cv::viz::makeCameraPose(
                cv::Vec3d(-camDist, 0, 0),
                cv::Vec3d(0, 0, 0),
                cv::Vec3d(0, 0, 1)
            )
        );

        // 说明文字
        auto hint = cv::viz::WText(
            "Orange arrow = rotated X-axis (forward direction)",
            cv::Point(10, 55), 14,
            cv::viz::Color(160, 160, 170)
        );
        window_.showWidget("hint_text", hint);
    }

    void addLabel(const std::string& id,
                  const std::string& text,
                  const cv::Point3d& pos,
                  const cv::viz::Color& color)
    {
        auto w = cv::viz::WText3D(text, pos, arrow_length_ * 0.10, false, color);
        window_.showWidget(id, w);
    }

    void addGrid()
    {
        double half = arrow_length_ * 1.6;
        double step = arrow_length_ * 0.4;
        cv::viz::Color gc(50, 50, 65);
        int idx = 0;
        for (double t = -half; t <= half + 1e-9; t += step)
        {
            std::vector<cv::Point3d> lx = {{t, -half, 0}, {t, half, 0}};
            window_.showWidget("gx" + std::to_string(idx),
                               cv::viz::WPolyLine(lx, gc));

            std::vector<cv::Point3d> ly = {{-half, t, 0}, {half, t, 0}};
            window_.showWidget("gy" + std::to_string(idx),
                               cv::viz::WPolyLine(ly, gc));
            ++idx;
        }
    }

    // ------------------------------------------------------------------------
    // 工具
    // ------------------------------------------------------------------------

    static cv::Vec3d toVec3d(const cv::Mat& rvec)
    {
        CV_Assert(!rvec.empty());
        cv::Mat tmp;
        rvec.convertTo(tmp, CV_64F);
        tmp = tmp.reshape(1, 3);
        if (tmp.rows != 3 || tmp.cols != 1)
            throw std::invalid_argument("rvec must be a 3-element vector");
        return cv::Vec3d(tmp.at<double>(0), tmp.at<double>(1), tmp.at<double>(2));
    }
};

// ============================================================================
// 便捷独立函数
// ============================================================================

/**
 * @brief 一行代码可视化 rvec（阻塞，关闭窗口后返回）
 */
inline void visualizeRvec(const cv::Vec3d&   rvec,
                          const std::string& window_name  = "Pose Arrow",
                          double             arrow_length = 0.25)
{
    RvecVisualizer vis(window_name, arrow_length);
    vis.show(rvec);
}

inline void visualizeRvec(const cv::Mat&     rvec,
                          const std::string& window_name  = "Pose Arrow",
                          double             arrow_length = 0.25)
{
    RvecVisualizer vis(window_name, arrow_length);
    vis.show(rvec);
}