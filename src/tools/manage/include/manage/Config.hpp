#pragma once
#include "yaml.hpp"

inline const std::string PROJECT_ROOT = YAML::getExecutableDir();

// 配置文件路径定义
inline YAML J_COMMON(PROJECT_ROOT + "/configs/config.yaml");

class Config
{
public:
    static Config &Instance()
    {
        static Config instance;
        return instance;
    }
};