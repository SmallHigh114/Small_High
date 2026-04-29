#ifndef RVEC_VISUALIZER_HPP
#define RVEC_VISUALIZER_HPP

/**
 * @file rvec_visualizer.hpp
 * @brief 使用 OpenCV Viz 可视化相机、云台、世界(Odom)三个坐标系
 *
 * 坐标系约定（右手系）:
 *   X → 前方   Y → 左方   Z → 上方
 *
 * 视角交互:
 *   鼠标左键拖动  → 旋转
 *   鼠标右键拖动  → 平移
 *   鼠标滚轮      → 缩放
 *   按 'q' / 'Q' → 退出
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// 旋转矩阵辅助（右手系，绕各轴正方向逆时针）
//   X→前，Y→左，Z→上
//   roll  绕X轴，pitch 绕Y轴，yaw 绕Z轴
// ─────────────────────────────────────────────────────────────────────────────
namespace rvec_detail {

// 绕 Z 轴（yaw）
inline cv::Matx33d Rz(double a) {
    double c=cos(a), s=sin(a);
    return {c,-s,0,  s,c,0,  0,0,1};
}
// 绕 Y 轴（pitch）
inline cv::Matx33d Ry(double a) {
    double c=cos(a), s=sin(a);
    return {c,0,s,  0,1,0,  -s,0,c};
}
// 绕 X 轴（roll）
inline cv::Matx33d Rx(double a) {
    double c=cos(a), s=sin(a);
    return {1,0,0,  0,c,-s,  0,s,c};
}

// ── keyboard callback：按 q/Q/Escape 退出 ──────────────────────────────────
struct QuitOnKey {
    static void callback(const cv::viz::KeyboardEvent & e, void * cookie) {
        if (e.action != cv::viz::KeyboardEvent::KEY_DOWN) return;
        char k = e.code;
        if (k == 'q' || k == 'Q' || k == 27 /* ESC */) {
            auto * win = reinterpret_cast<cv::viz::Viz3d*>(cookie);
            win->close();
        }
    }
};

} // namespace rvec_detail

// ─────────────────────────────────────────────────────────────────────────────
// 主可视化函数
// ─────────────────────────────────────────────────────────────────────────────
inline void visualizeRvec(
    const cv::Mat & odom_rvec,
    float cam2gimDis     = 0.1f,
    float gim2odom_angle = 1.57f,
    float gim2odomDis    = 0.5f,
    float gimbal_pitch   = -0.5236f,
    float gimbal_yaw     = -0.7854f,
    float roll           = 0.0f,
    double axis_length   = 0.5
)
{
    using namespace rvec_detail;

    // ── 1. 重建各坐标系原点（与 coordinateTransform 逻辑一致）──────────────

    // 相机光心系 → 相机坐标系（右手系：X前Y左Z上）
    //   OpenCV 光心系：Z前、X右、Y下
    //   变换后：X_cam = Z_photo（前），Y_cam = -X_photo（左），Z_cam = -Y_photo（上）
    cv::Matx33d R_cam_photo(
         0,  0,  1,
        -1,  0,  0,
         0, -1,  0);

    cv::Matx33d R_pitch       = Ry(static_cast<double>(gimbal_pitch));
    cv::Matx33d R_yaw         = Rz(static_cast<double>(gimbal_yaw));
    cv::Matx33d R_roll        = Rx(static_cast<double>(roll));
    cv::Matx33d R_odom_gimbal = R_yaw * R_roll;   // 云台→世界旋转

    // 世界系下云台原点（连杆末端）
    cv::Vec3d t_odom(
        static_cast<double>(gim2odomDis) * std::cos(static_cast<double>(gim2odom_angle)),
        0.0,
        static_cast<double>(gim2odomDis) * std::sin(static_cast<double>(gim2odom_angle)));
    cv::Vec3d P_gimbal_world = R_odom_gimbal * t_odom;

    // 相机原点在云台系 = R_pitch * t_gimbal
    cv::Vec3d t_gimbal(static_cast<double>(cam2gimDis), 0.0, 0.0);
    cv::Vec3d P_cam_world = R_odom_gimbal * (R_pitch * t_gimbal + t_odom);

    cv::Vec3d P_world(0.0, 0.0, 0.0);

    // ── 2. 控制台诊断 ─────────────────────────────────────────────────────
    std::cout << "\n=== [Viz] 各坐标系原点（世界系，右手系 X前Y左Z上）===\n";
    std::cout << "World  : (0, 0, 0)\n";
    std::cout << "Gimbal : ("<<P_gimbal_world[0]<<", "<<P_gimbal_world[1]<<", "<<P_gimbal_world[2]<<")\n";
    std::cout << "Camera : ("<<P_cam_world[0]   <<", "<<P_cam_world[1]   <<", "<<P_cam_world[2]   <<")\n";
    std::cout << "axis_length = " << axis_length << " m\n";
    std::cout << "====================================================\n\n";

    // ── 3. 构建 Affine3d 位姿 ─────────────────────────────────────────────
    cv::Affine3d pose_world (cv::Matx33d::eye(),  P_world);
    cv::Affine3d pose_gimbal(R_odom_gimbal,        P_gimbal_world);
    cv::Affine3d pose_camera(R_odom_gimbal * R_pitch * R_cam_photo, P_cam_world);

    cv::Mat R_target_mat;
    cv::Rodrigues(odom_rvec, R_target_mat);
    cv::Matx33d R_target;
    for (int r=0;r<3;++r) for(int c=0;c<3;++c)
        R_target(r,c)=R_target_mat.at<double>(r,c);
    cv::Affine3d pose_target(R_target, cv::Vec3d(0,0,0));

    // ── 4. 创建窗口 ───────────────────────────────────────────────────────
    cv::viz::Viz3d win("Coordinate Systems  [右手系: X前 Y左 Z上]");
    win.setWindowSize(cv::Size(1280, 720));
    win.setBackgroundColor(cv::viz::Color(25, 25, 35));

    // ── 5. 注册键盘回调（解决按 q 无反应问题）─────────────────────────────
    win.registerKeyboardCallback(QuitOnKey::callback, &win);

    // ── 6. 坐标轴组件 ─────────────────────────────────────────────────────
    double al = axis_length;

    // 世界坐标系（最大，标准 RGB 轴：红X绿Y蓝Z）
    win.showWidget("w_axes",  cv::viz::WCoordinateSystem(al),         pose_world);
    win.showWidget("w_label", cv::viz::WText3D("World (Odom)",
        cv::Point3d(0, 0, al*1.15), al*0.14, false, cv::viz::Color::white()), pose_world);

    // 云台坐标系（黄色标签）
    win.showWidget("g_axes",  cv::viz::WCoordinateSystem(al*0.8),     pose_gimbal);
    win.showWidget("g_label", cv::viz::WText3D("Gimbal",
        cv::Point3d(0, 0, al*1.05), al*0.12, false, cv::viz::Color::yellow()), pose_gimbal);

    // 相机坐标系（青色标签）
    win.showWidget("c_axes",  cv::viz::WCoordinateSystem(al*0.65),    pose_camera);
    win.showWidget("c_label", cv::viz::WText3D("Camera",
        cv::Point3d(0, 0, al*0.85), al*0.11, false, cv::viz::Color::cyan()), pose_camera);

    // 目标姿态（绿色标签，位于世界原点）
    win.showWidget("t_axes",  cv::viz::WCoordinateSystem(al*0.5),     pose_target);
    win.showWidget("t_label", cv::viz::WText3D("Target",
        cv::Point3d(0, 0, al*0.65), al*0.10, false, cv::viz::Color::green()), pose_target);

    // ── 7. 变换链连接线 ───────────────────────────────────────────────────
    auto makeLine = [&](cv::Vec3d a, cv::Vec3d b, cv::viz::Color col,
                        const std::string & id) {
        std::vector<cv::Point3d> pts = {
            cv::Point3d(a[0],a[1],a[2]), cv::Point3d(b[0],b[1],b[2])};
        cv::viz::WPolyLine line(pts, col);
        line.setRenderingProperty(cv::viz::LINE_WIDTH, 2.5);
        win.showWidget(id, line);
    };
    makeLine(P_world,        P_gimbal_world, cv::viz::Color::yellow(), "line_wg");
    makeLine(P_gimbal_world, P_cam_world,    cv::viz::Color::cyan(),   "line_gc");

    // ── 8. 原点球体（更容易找到各坐标系位置）────────────────────────────
    double sr = al * 0.045;
    win.showWidget("sp_w", cv::viz::WSphere(cv::Point3d(0,0,0), sr, 12,
        cv::viz::Color::white()));
    win.showWidget("sp_g", cv::viz::WSphere(
        cv::Point3d(P_gimbal_world[0],P_gimbal_world[1],P_gimbal_world[2]), sr, 12,
        cv::viz::Color::yellow()));
    win.showWidget("sp_c", cv::viz::WSphere(
        cv::Point3d(P_cam_world[0],P_cam_world[1],P_cam_world[2]), sr, 12,
        cv::viz::Color::cyan()));

    // ── 9. 右手系参考网格（XY 平面 = 水平地面，Z 朝上）──────────────────
    // WGrid 默认在 XY 平面。Viz 内部坐标 = 右手系，与我们约定一致。
    cv::viz::WGrid grid(cv::Vec2i(20, 20), cv::Vec2d(0.2, 0.2),
                        cv::viz::Color(55, 55, 55));
    win.showWidget("grid", grid);

    // 在 X/Y/Z 轴端点加箭头标注（帮助确认右手系方向）
    auto makeArrowLabel = [&](cv::Vec3d from, cv::Vec3d to,
                              cv::viz::Color col, const std::string & label,
                              const std::string & id) {
        // 箭头
        win.showWidget(id+"_arr",
            cv::viz::WArrow(cv::Point3d(from[0],from[1],from[2]),
                            cv::Point3d(to[0],  to[1],  to[2]),
                            0.005, col));
        // 文字
        win.showWidget(id+"_txt",
            cv::viz::WText3D(label,
                cv::Point3d(to[0], to[1], to[2]+al*0.08),
                al*0.12, false, col));
    };
    double gl = al * 1.35;  // 全局轴箭头长度（稍长于坐标轴）
    makeArrowLabel({0,0,0},{gl,0,0},  cv::viz::Color::red(),   "X (前)", "ax_x");
    makeArrowLabel({0,0,0},{0,gl,0},  cv::viz::Color::green(), "Y (左)", "ax_y");
    makeArrowLabel({0,0,0},{0,0,gl},  cv::viz::Color::blue(),  "Z (上)", "ax_z");

    // ── 10. HUD 文字 ──────────────────────────────────────────────────────
    win.showWidget("hud_op",
        cv::viz::WText(
            "Left drag: Rotate  |  Right drag: Pan  |  Scroll: Zoom  |  Q: Quit",
            cv::Point(10, 22), 15, cv::viz::Color(200, 200, 200)));
    win.showWidget("hud_legend",
        cv::viz::WText(
            "White=World(Odom)   Yellow=Gimbal   Cyan=Camera   Green=Target",
            cv::Point(10, 48), 14, cv::viz::Color(160, 200, 160)));
    win.showWidget("hud_cs",
        cv::viz::WText(
            "Right-hand frame: X=Forward(Red)  Y=Left(Green)  Z=Up(Blue)",
            cv::Point(10, 72), 13, cv::viz::Color(140, 180, 220)));

    // ── 11. 设置初始视角（makeCameraPose 保证正确） ───────────────────────
    //   从右前上方俯瞰原点，up = Z 轴
    win.setViewerPose(
        cv::viz::makeCameraPose(
            cv::Vec3d(2.0, -1.5, 1.8),   // eye
            cv::Vec3d(0.0,  0.0, 0.0),   // 看向原点
            cv::Vec3d(0.0,  0.0, 1.0)    // Z 朝上
        )
    );

    // ── 12. 主循环 ────────────────────────────────────────────────────────
    //
    //  【交互问题根本原因】
    //  spinOnce(ms, force_redraw) 的第二个参数 force_redraw=true 会强制每帧
    //  重绘，但在某些 VTK 后端下反而阻止了事件队列处理。
    //  正确做法：
    //    - 用 spin() 阻塞直到窗口关闭（最简单，完全交互）
    //    - 或 spinOnce(1, false) 快速轮询（force_redraw=false）
    //
    //  这里用 spin()，交互由 VTK 内部 interactor 全权处理。
    //  键盘 q/Q/ESC → registerKeyboardCallback 里调用 win.close()。
    //
    std::cout << "[Viz] 窗口已打开 — 鼠标旋转/平移/缩放，按 Q 退出\n";
    win.spin();   // ← 阻塞，完全交互，替代 while(!wasStopped()) spinOnce
}

#endif // RVEC_VISUALIZER_HPP