# Coordinate_Transform

一个基于 Eigen3 和 yaml-cpp 实现的轻量级高频坐标变换（TF）树管理库。该库不依赖 ROS 或者是分布式 IPC 环境，支持单体项目、本地标定以及嵌入式机器人控制系统中的坐标系时空网络构建。

---

## 🛠️ 依赖要求

- **C++ 标准**: 不低于 C++20
- **依赖**: 
  - [Eigen3](https://eigen.tuxfamily.org/)
  - [yaml-cpp](https://github.com/jbeder/yaml-cpp)

---

## 📄 配置文件格式 (YAML)

你可以通过统一的配置文件来管理机器人所有的静态或动态外参坐标系。以下是库支持的规范 YAML 格式示例：

```yaml
static_broudcaster_conf:
  transforms:
    - parent_frame_id: "map"
      child_frame_id: "odom"
      translation:
        x: 1.0
        y: 2.0
        z: 0.0
      rpy_angle:
        roll: 0.0
        pitch: 0.0
        yaw: 45.0

    - parent_frame_id: "odom"
      child_frame_id: "gimbal"
      translation:
        x: 0.1
        y: 0.0
        z: 0.5
      rpy_angle:
        roll: 0.0
        pitch: 10.0
        yaw: 0.0

    - parent_frame_id: "gimbal"
      child_frame_id: "camera"
      translation:
        x: 0.05
        y: 0.03
        z: 0.02
      rpy_angle:
        roll: -90.0
        pitch: 0.0
        yaw: -90.0
```

---

## 🚀 构建 TF 变换树

本系统支持通过批量载入配置或动态输入的位姿数据，隐式在底层织入一张有向无环的**坐标变换拓扑树（TF Tree）**。用户只需关心局部的父子坐标系相对关系，系统将自动解算任意两个孤立坐标系之间的级联变换。

### 1️⃣ 位姿数据打包

将外部解析出的平移向量（YAML 或传感器数据）与旋转四元数（通过欧拉角或轴角转换）统一封装至拓扑单元 `tf::Coordinate` 中：

```cpp
tf::Coordinate cod; 

Eigen::Vector3d translation(x, y, z);
Eigen::Quaterniond rotation = tf::getQuaterniond(rpy_angle);

cod.translation = translation;
cod.rotation    = rotation;

```

### 2️⃣ 批量遍历与树构建

通过循环遍历配置节点，调用 `CoordinateInit` 接口将节点依次压入缓冲区，系统会自动在内存中完成拓扑“织网”：

```cpp
// 遍历 transforms_node 数组，一键构建多层级变换树
for (const auto& node : transforms_node) {
    // ... [解析 node 参数并打包至 cod] ...
    
    tf::CoordinateInit(buf, cod, true);
}

```

---

## 🔍 任意坐标系动态解算 (.get)

当变换树构建完成后，内部的拓扑图连通性机制激活。用户不再需要手动级联繁琐的矩阵乘法，只需调用 `buf.get()` 即可获取**任意两个存在连通路径的坐标系之间**的齐次变换矩阵。

### 接口原型

```cpp
/**
 * @brief 动态追溯拓扑路径，计算 source 坐标系到 target 坐标系的齐次变换矩阵
 * @param _target_frame 目标坐标系 (变换后的参考基准)
 * @param _source_frame 初始坐标系 (待转换的点所在的原始参考系)
 * @param _query_time   检索时间戳 (用于动态 TF 线性插值)
 * @param _tolerance    时间戳匹配的抖动容忍度/超时时间
 * @return Eigen::Isometry3d 4x4 齐次变换矩阵 
 */
Eigen::Isometry3d transform_buffer::get(
    const std::string& _target_frame,
    const std::string& _source_frame,
    const time_t& _query_time,
    const duration_t& _tolerance
) const;

```

### 调用示例

获取将数据从 **云台坐标系 (gimbal)** 变换到 **相机坐标系 (camera)** 的变换矩阵：

```cpp
try {
    auto stamp = std::chrono::system_clock::now();
    
    // 获取 4x4 变换矩阵 T_camera_from_gimbal
    Eigen::Isometry3d T_camera_to_gimbal = buf.get(
        "camera",                       // Target Frame
        "gimbal",                       // Source Frame
        stamp,                          // Query Time
        std::chrono::milliseconds(10)   // 时间容忍度 (10ms)
    );
    
    // 打印标准 4x4 矩阵
    std::cout << "Transform Matrix:\n" << T_camera_to_gimbal.matrix() << std::endl;

} catch (const std::exception& e) {
    std::cerr << "TF Lookup Failed: " << e.what() << std::endl;
}

```

---

## 📐 数学原理

在机器人学中，坐标变换矩阵的实际应用遵循“从右向左”的齐次坐标变换原则（左乘）。

假设我们在 **gimbal (云台)** 坐标系下通过雷达或传感器测得了一个物体的三维坐标点 $P_{gimbal}$。若想知道该点在 **camera (相机)** 坐标系下的投影或三维坐标 $P_{camera}$，必须使用上面获取的 `T_camera_to_gimbal` 矩阵对其进行**标准左乘**。

### 数学映射公式

$$P_{camera} = T_{camera\_from\_gimbal} \times P_{gimbal}$$

### C++ 实现

```cpp
Eigen::Vector3d P_gimbal(1.0, 0.5, -0.2);

Eigen::Vector3d P_camera = T_camera_to_gimbal * P_gimbal;

std::cout << "Point in gimbal frame: " << P_gimbal.transpose() << std::endl;
std::cout << "Point transformed to camera frame: " << P_camera.transpose() << std::endl;

```
