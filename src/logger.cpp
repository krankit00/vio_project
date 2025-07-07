#include "logger.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

PerformanceLogger::PerformanceLogger(const std::string& log_filename) {
    log_file_.open(log_filename);
    if (log_file_.is_open()) {
        // Write header
        log_file_ << "Timestamp (ms),CPU (%),RAM (MB)\n";
        start_time_ = std::chrono::steady_clock::now();
        last_total_time_ = getTotalCPUTime();
        last_process_time_ = getProcessCPUTime();
        std::cout << "Performance logger initialized, writing to " << log_filename << std::endl;
    } else {
        std::cerr << "ERROR: Could not open log file: " << log_filename << std::endl;
    }
}

PerformanceLogger::~PerformanceLogger() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void PerformanceLogger::log() {
    if (!log_file_.is_open()) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    long long timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();

    // RAM Usage
    double ram_mb = static_cast<double>(getProcessMemory()) / 1024.0;

    // CPU Usage
    long current_total_time = getTotalCPUTime();
    long current_process_time = getProcessCPUTime();

    double cpu_percent = 0.0;
    long total_time_delta = current_total_time - last_total_time_;
    long process_time_delta = current_process_time - last_process_time_;

    if (total_time_delta > 0) {
        cpu_percent = 100.0 * static_cast<double>(process_time_delta) / static_cast<double>(total_time_delta);
    }
    
    log_file_ << timestamp << "," << cpu_percent << "," << ram_mb << "\n";

    // Update for next calculation
    last_total_time_ = current_total_time;
    last_process_time_ = current_process_time;
}


// --- Linux-Specific Implementations ---

// Returns process CPU time in clock ticks (jiffies)
long PerformanceLogger::getProcessCPUTime() {
    std::ifstream stat_file("/proc/self/stat");
    if (!stat_file) return 0;
    std::string line;
    std::getline(stat_file, line);
    
    std::istringstream ss(line);
    std::string value;
    long utime = 0, stime = 0;

    // Fields are 1-based. We need 14 (utime) and 15 (stime).
    for (int i = 1; i <= 15; ++i) {
        ss >> value;
        if (i == 14) utime = std::stol(value);
        if (i == 15) stime = std::stol(value);
    }
    return utime + stime;
}

// Returns total system CPU time in clock ticks (jiffies)
long PerformanceLogger::getTotalCPUTime() {
    std::ifstream stat_file("/proc/stat");
    if (!stat_file) return 0;
    std::string line;
    std::getline(stat_file, line); // First line is 'cpu' aggregate

    std::istringstream ss(line);
    std::string cpu_label;
    ss >> cpu_label; // "cpu"
    long total_time = 0, val;
    while(ss >> val) {
        total_time += val;
    }
    return total_time;
}

// Returns process memory usage (Resident Set Size) in KB
long PerformanceLogger::getProcessMemory() {
    std::ifstream status_file("/proc/self/status");
    if (!status_file) return 0;
    std::string line;
    long memory = 0;
    while (std::getline(status_file, line)) {
        if (line.rfind("VmRSS:", 0) == 0) { // Starts with VmRSS:
            std::istringstream ss(line);
            std::string label;
            ss >> label >> memory;
            break;
        }
    }
    return memory;
}