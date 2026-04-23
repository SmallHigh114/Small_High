# 无人机反制 lidar-against_drones
## 架构
``` text
lidar-against_drones
├── docs/                       # 存放测试视频
├── models/                     # 存放模型文件
├── src/                        
│   ├── build/                  # 编译文件
│   │   ├── run/                # 可执行文件
│   │   ├── ...
│   │   └── Makefile
│   ├── core/                    # 核心算法层
│   │   ├── detector/            # 检测器
│   │   ├── filter/              # 一阶低通滤波
│   │   ├── pnp_solver/          # pnp解算
│   │   ├── yolo/                # 神经网络
│   │   └── README.md
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   ├── main.cpp             # main 
│   │   └── README.md
│   ├── tools/                   # 工具
│   │   ├── hikcamera/           # hik SDK
│   │   ├── ui/                  # ui
│   │   └── README.md
│   ├── CMakeLists.txt
│   ├── .gitignore
│   └── README.md
```
## 运行环境
- `OpenCV` - V4.8.0
- `OpenVINO ` - v2024.1.0
- `CMake` - v3.28.3