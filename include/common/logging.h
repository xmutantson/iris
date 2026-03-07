#ifndef IRIS_LOGGING_H
#define IRIS_LOGGING_H

#include <cstdio>
#include <cstdarg>
#include <string>
#include <mutex>
#include <chrono>

namespace iris {

class Logger {
public:
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    // Open log file (tee: stdout + file). Returns false on failure.
    bool open(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_) fclose(file_);
        file_ = fopen(path.c_str(), "a");
        if (!file_) return false;
        log_path_ = path;

        // Write header
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&t));
        fprintf(file_, "\n=== Iris log started %s ===\n", buf);
        fflush(file_);
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_) {
            fclose(file_);
            file_ = nullptr;
        }
    }

    void log(const char* fmt, ...) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Timestamp
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_).count();

        char prefix[32];
        snprintf(prefix, sizeof(prefix), "[%6lld.%03lld] ",
                 (long long)(ms / 1000), (long long)(ms % 1000));

        va_list args;

        // Print to stdout
        fputs(prefix, stdout);
        va_start(args, fmt);
        vfprintf(stdout, fmt, args);
        va_end(args);
        fputc('\n', stdout);
        fflush(stdout);

        // Print to file
        if (file_) {
            fputs(prefix, file_);
            va_start(args, fmt);
            vfprintf(file_, fmt, args);
            va_end(args);
            fputc('\n', file_);
            fflush(file_);
        }
    }

    bool is_open() const { return file_ != nullptr; }

private:
    Logger() : file_(nullptr), start_(std::chrono::steady_clock::now()) {}
    ~Logger() { close(); }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    FILE* file_;
    std::string log_path_;
    std::mutex mutex_;
    std::chrono::steady_clock::time_point start_;
};

} // namespace iris

// Convenience macro
#define IRIS_LOG(...) iris::Logger::instance().log(__VA_ARGS__)

#endif
