#pragma once
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace fast_livo2 {
// Optional diagnostics use one bounded file and never perform disk I/O in write().
class RuntimeLog {
public:
  explicit RuntimeLog(bool enabled);
  ~RuntimeLog();
  void write(std::string row);

private:
  void run();
  bool enabled_, stopping_ = false;
  size_t queued_bytes_ = 0, dropped_ = 0;
  std::deque<std::string> queue_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::thread worker_;
};
} // namespace fast_livo2
