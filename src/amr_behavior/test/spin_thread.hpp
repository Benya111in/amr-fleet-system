// 시험용 실행기 스레드: 시작 직후 cancel() 이 spin() 보다 먼저 불려도 확실히 멈춘다. (rclcpp
// Executor::spin() 은 시작 시 spinning 플래그를 다시 세우므로 한 번의 cancel 로는 경합이 남는다)
#ifndef SPIN_THREAD_HPP_
#define SPIN_THREAD_HPP_

#include <atomic>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace amr_behavior_test
{

class SpinThread
{
public:
  explicit SpinThread(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {
        executor_.spin();
        done_ = true;
      }) {}

  ~SpinThread() {stop();}

  void stop()
  {
    if (!thread_.joinable()) {
      return;
    }
    while (!done_) {
      executor_.cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    thread_.join();
  }

private:
  rclcpp::Executor & executor_;
  std::atomic<bool> done_{false};
  std::thread thread_;
};

}  // namespace amr_behavior_test

#endif  // SPIN_THREAD_HPP_
