#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <fstream>
#include <streambuf>
#include <ctime>
#include <iomanip>
#include <sstream>
/**
 * 捕获日志
 */
namespace debug
{
// ================= TeeBuf：双输出缓冲区 =================
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* sb1, std::streambuf* sb2)
        : sb1_(sb1), sb2_(sb2) {}

protected:
    virtual int overflow(int c) override {
        if (c == EOF) return !EOF;
        if (sb1_->sputc(c) == EOF || sb2_->sputc(c) == EOF)
            return EOF;
        return c;
    }

    virtual int sync() override {
        return (sb1_->pubsync() == 0 && sb2_->pubsync() == 0) ? 0 : -1;
    }

private:
    std::streambuf* sb1_;
    std::streambuf* sb2_;
};

// ================= 时间戳工具 =================
inline std::string getCurrentTime() {
    auto now = std::time(nullptr);
    std::tm* tm = std::localtime(&now);

    std::ostringstream oss;
    oss << std::put_time(tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

// ================= 带时间戳输出缓冲 =================
class TimeStampBuf : public std::streambuf {
public:
    TimeStampBuf(std::streambuf* dest) : dest_(dest), newLine_(true) {}

protected:
    virtual int overflow(int c) override {
        if (c == EOF) return !EOF;

        if (newLine_) {
            std::string ts = "[" + getCurrentTime() + "] ";
            dest_->sputn(ts.c_str(), ts.size());
            newLine_ = false;
        }

        if (c == '\n') newLine_ = true;

        return dest_->sputc(c);
    }

private:
    std::streambuf* dest_;
    bool newLine_;
};

// ================= Logger 类 =================
class Logger {
public:
    Logger(const std::string& prefix = "log") {
        // 生成文件名
        auto now = std::time(nullptr);
        std::tm* tm = std::localtime(&now);

        std::ostringstream filename;
        filename << prefix << "_"
                 << std::put_time(tm, "%Y%m%d_%H%M%S")
                 << ".txt";

        file_.open(filename.str());

        if (!file_.is_open()) {
            std::cerr << "Failed to open log file!" << std::endl;
            return;
        }

        // 保存原始 buffer
        oldCout_ = std::cout.rdbuf();
        oldCerr_ = std::cerr.rdbuf();

        // 构建带时间戳的 buffer
        tsBuf_ = new TimeStampBuf(file_.rdbuf());

        // 构建双输出
        teeBuf_ = new TeeBuf(oldCout_, tsBuf_);

        // 重定向
        std::cout.rdbuf(teeBuf_);
        std::cerr.rdbuf(teeBuf_);
    }

    ~Logger() {
        // 恢复
        std::cout.rdbuf(oldCout_);
        std::cerr.rdbuf(oldCerr_);

        if (file_.is_open())
            file_.close();

        delete teeBuf_;
        delete tsBuf_;
    }

private:
    std::ofstream file_;

    std::streambuf* oldCout_;
    std::streambuf* oldCerr_;

    TeeBuf* teeBuf_;
    TimeStampBuf* tsBuf_;
};
}


#endif // LOGGER_HPP