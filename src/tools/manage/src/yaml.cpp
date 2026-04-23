// yaml.cpp
#include "manage/yaml.hpp"

#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <ctime>

bool YAML::verifyModifiedTime()
{
    struct stat file_state{};
    if (stat(this->path_.c_str(), &file_state) != 0)
    {
        std::cout << "yaml file path stat error." << std::endl;
        return false;
    }

    std::time_t modified_time = file_state.st_mtime;

    if (this->last_modified_time_ == 0 && this->last_modified_time_ != modified_time)
    {
        this->last_modified_time_ = modified_time;
        return false;
    }

    bool status = (this->last_modified_time_ != modified_time);
    this->last_modified_time_ = modified_time;
    return status;
}

void YAML::updateJson()
{
    if (this->verifyModifiedTime())
    {
        config_.release();
        config_.open(this->path_, cv::FileStorage::READ);
        std::cout << "json config changed." << std::endl;
    }
}

std::string YAML::getExecutableDir()
{
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len != -1)
    {
        buf[len] = '\0';
        std::string path(buf);
        return path.substr(0, path.find_last_of('/'));
    }
    return "."; // fallback
}

YAML::~YAML()
{
    this->config_.release();
}
