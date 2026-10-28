#include "runtime_log.h"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fast_livo2 {
RuntimeLog::RuntimeLog(bool enabled) : enabled_(enabled) {
  if (enabled_)
    worker_ = std::thread(&RuntimeLog::run, this);
}
RuntimeLog::~RuntimeLog() {
  if (!enabled_)
    return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  changed_.notify_one();
  worker_.join();
}
void RuntimeLog::write(std::string row) {
  if (!enabled_)
    return;
  row += '\n';
  std::lock_guard<std::mutex> lock(mutex_);
  if (queued_bytes_ + row.size() > 1024 * 1024) {
    ++dropped_;
    return;
  }
  queued_bytes_ += row.size();
  queue_.push_back(std::move(row));
}
void RuntimeLog::run() {
  const char *configured = std::getenv("FAST_LIVO2_LOG_DIR");
  const char *home = std::getenv("HOME");
  const auto directory = configured
                             ? std::filesystem::path(configured)
                             : std::filesystem::path(home ? home : ".") / ".ros/fast_livo2/logs";
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  const auto file = directory / "runtime.log";
  size_t bytes = std::filesystem::exists(file, error) ? std::filesystem::file_size(file, error) : 0;
  std::ofstream stream;
  if (!error)
    stream.open(file, std::ios::app);
  bool failed = error || !stream;
  size_t quota_drops = 0;
  for (;;) {
    std::deque<std::string> batch;
    bool stopping;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait_for(lock, std::chrono::milliseconds(250), [this] { return stopping_; });
      batch.swap(queue_);
      queued_bytes_ = 0;
      stopping = stopping_;
    }
    std::string text;
    for (const auto &row : batch) {
      if (failed || bytes + text.size() + row.size() > 100000000)
        ++quota_drops;
      else
        text += row;
    }
    if (!text.empty()) {
      stream.write(text.data(), text.size());
      stream.flush();
      if (!stream)
        failed = true;
      else
        bytes += text.size();
    }
    if (stopping)
      break;
  }
  if (failed || dropped_ || quota_drops)
    std::cerr << "FAST-LIVO2 runtime log: write_failed=" << failed << " queue_dropped=" << dropped_
              << " storage_dropped=" << quota_drops << '\n';
}
} // namespace fast_livo2
