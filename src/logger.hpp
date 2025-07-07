#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <fstream>
#include <chrono>

class PerformanceLogger {
public:
    PerformanceLogger(const std::string& log_filename);
    ~PerformanceLogger();

    // Logs current CPU and RAM usage to the file
    void log();

private:
    std::ofstream log_file_;
    std::chrono::steady_clock::time_point start_time_;

    // For CPU calculation
    long last_total_time_ = 0;
    long last_process_time_ = 0;

    // Platform-specific functions
    long getProcessCPUTime(); // In jiffies
    long getTotalCPUTime();   // In jiffies
    long getProcessMemory();  // In KB
};

#endif // LOGGER_HPP