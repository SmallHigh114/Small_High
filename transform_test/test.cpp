#include <iostream>
#include <chrono>
#include <numbers>
#include <filesystem>

#include <yaml-cpp/yaml.h>

// #include "transform_coordinate/transform_coordinate.hpp"
// #include "transform_coordinate/rvec_visualizer.hpp"

#include "transform_coordinate/fast_tf.hpp"

/// yaml
/// transforms:
///   - parent_frame_id: map
///     child_frame_id: odom
///     translation:
///       x: 0.0
///       y: 0.0
///       z: 0.0
///     rpy_angle:
///       roll: 0.0
///       pitch: 0.0
///       yaw: 0.0

namespace tf = fast_tf::detail;

int main() {

    std::filesystem::path source_file_path(__FILE__);
        
    std::filesystem::path yaml_path = source_file_path.parent_path() / "config" / "config.yaml";

    std::string yaml_file_path = yaml_path.string();
    std::cout << "Loading config from: " << yaml_file_path << std::endl;
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_file_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
        return -1;
    }

    auto transforms_node = config["static_broudcaster_conf"]["transforms"];
    auto stamp = std::chrono::system_clock::now();

    tf::transform_buffer buf;
    tf::Coordinate cod;
    tf::RpyAngle rpy_angle;

    for (const auto& node : transforms_node) {
        cod.stamp = std::chrono::system_clock::now();
        cod.parent_frame_id = node["parent_frame_id"].as<std::string>();
        cod.child_frame_id = node["child_frame_id"].as<std::string>();

        cod.translation.x() = node["translation"]["x"].as<double>();
        cod.translation.y() = node["translation"]["y"].as<double>();
        cod.translation.z() = node["translation"]["z"].as<double>();

        rpy_angle.pitch = node["rpy_angle"]["pitch"].as<double>();
        rpy_angle.roll = node["rpy_angle"]["roll"].as<double>();
        rpy_angle.yaw = node["rpy_angle"]["yaw"].as<double>();

        cod.rotation = tf::getQuaterniond(rpy_angle);

        tf::CoordinateInit(buf, cod, true);
    }

    Eigen::Isometry3d T_camera_to_gimbal = buf.get("camera","gimbal", stamp, std::chrono::milliseconds(10));
    std::cout << T_camera_to_gimbal.matrix() << std::endl;

    return 0;
}

// int main()
// {
//     auto start = std::chrono::high_resolution_clock::now();

//     cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
//     cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
//     cv::Mat odom_tvec = cv::Mat::zeros(3, 1, CV_64F);
//     cv::Mat odom_rvec = cv::Mat::zeros(3, 1, CV_64F);

//     float cam2gimDis = 0.1;
//     float gim2odom_angle = 0.7853;
//     float gim2odomDis = 1.414;
//     float gimbal_pitch = 0.2617;
//     float gimbal_yaw = 0.7853;
//     float roll = 0;

//     tvec.at<double>(0,0) = 0;
//     tvec.at<double>(1,0) = -1;
//     tvec.at<double>(2,0) = 1;
//     rvec.at<double>(0,0) = 0;
//     rvec.at<double>(1,0) = -0.7853;
//     rvec.at<double>(2,0) = 0;

//     TransForm::coordinateTransform(
//         tvec,
//         rvec,
//         cam2gimDis, 
//         gim2odom_angle, 
//         gim2odomDis, 
//         gimbal_pitch, 
//         gimbal_yaw,
//         odom_tvec,
//         odom_rvec,
//         roll
//     );

//     std::cout << "odom_tvec = " << std::endl;
//     std::cout << odom_tvec << std::endl;
//     std::cout << "odom_rvec = " << std::endl;
//     std::cout << odom_rvec << std::endl;

//     auto end = std::chrono::high_resolution_clock::now();

//     double duration_ms =
//         std::chrono::duration<double, std::milli>(end - start).count();

//     std::cout << "----------------------------------" << std::endl;
//     std::cout << "Total time: " << duration_ms << " ms" << std::endl;

//     visualizeRvec(
//     odom_rvec,
//     cam2gimDis, gim2odom_angle, gim2odomDis,
//     gimbal_pitch, gimbal_yaw, roll
// );  
//     return 0;
// }

// void loadAndPackTransforms(const std::string& yaml_file_path, tf_detail::transform_buffer& buffer) {
//     // 1. 加载 YAML 文件
//     YAML::Node config;
//     try {
//         config = YAML::LoadFile(yaml_file_path);
//     } catch (const std::exception& e) {
//         std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
//         return;
//     }

//     // 2. 检查顶层键名是否存在
//     if (!config["static_broudcaster_conf"] || !config["static_broudcaster_conf"]["transforms"]) {
//         std::cerr << "Invalid config format: missing 'transforms'" << std::endl;
//         return;
//     }

//     // 3. 获取当前时间戳（代替原先从消息头获取的时间）
//     auto current_stamp = std::chrono::system_clock::now();

//     // 4. 获取 transforms 数组
//     auto transforms_node = config["static_broudcaster_conf"]["transforms"];

//     // 5. 遍历数组，一个一个解包并直接塞进 buffer_
//     for (const auto& node : transforms_node) {
//         try {
//             // 提取基础字符串
//             std::string parent_id = node["parent_frame_id"].as<std::string>();
//             std::string child_id = node["child_frame_id"].as<std::string>();

//             // 提取平移参数 (translation)
//             Eigen::Vector3d translation;
//             translation.x() = node["translation"]["x"].as<double>();
//             translation.y() = node["translation"]["y"].as<double>();
//             translation.z() = node["translation"]["z"].as<double>();

//             // 提取欧拉角并计算四元数 (rpy_angle)
//             double roll  = node["rpy_angle"]["roll"].as<double>()  * std::numbers::pi / 180.0;
//             double pitch = node["rpy_angle"]["pitch"].as<double>() * std::numbers::pi / 180.0;
//             double yaw   = node["rpy_angle"]["yaw"].as<double>()   * std::numbers::pi / 180.0;

//             Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
//             Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
//             Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
//             Eigen::Quaterniond rotation(yaw_angle * pitch_angle * roll_angle);
//             rotation.normalize();

//             // 使用你之前找到的数学逻辑构建 Isometry3d Matrix
//             Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
//             I.prerotate(rotation);
//             I.translate(translation); // 旋转后的坐标系作为基准来移动

//             // 6. 核心：直接塞进本地的 buffer_（省去了冰羚发送、排队、接收的全部过程）
//             buffer.set(parent_id, child_id, current_stamp, I, true);

//             std::cout << "Successfully packed static TF: " << parent_id << " -> " << child_id << std::endl;

//         } catch (const std::exception& e) {
//             std::cerr << "Error parsing a transform node: " << e.what() << std::endl;
//             // 某一个节点出错可以选择跳过，继续解析下一个
//             continue; 
//         }
//     }
// }