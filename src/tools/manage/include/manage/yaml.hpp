#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class YAML
{
public:
    template <typename T>
    YAML(const T &path) : path_(path)
    {
        config_.open(path_, cv::FileStorage::READ);
    }
    ~YAML();

    cv::FileStorage config_;
    void updateJson();
    static std::string getExecutableDir();

private:
    std::string path_;
    time_t last_modified_time_ = 0;
    bool verifyModifiedTime();
};