// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TILDE_TIMING_MONITOR__TILDE_TIMING_MONITOR_CORE_HPP_
#define TILDE_TIMING_MONITOR__TILDE_TIMING_MONITOR_CORE_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "tilde_timing_monitor_interfaces/msg/tilde_timing_monitor_deadline_miss.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <deque>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace tilde_timing_monitor
{
struct DeadlineTimer
{
  int64_t self_j;
  double start_time;
  double timer_val;
  rclcpp::TimerBase::SharedPtr timer;
  uint64_t uniq;
  bool valid;
};

using DeadlineTimerMap = std::unordered_map<uint64_t, DeadlineTimer>;

enum class e_stat
{
  ST_NONE,
  ST_INIT,
  ST_DETECT,
};

class TildePathConfig
{
public:
  uint32_t index;
  std::string path_name;
  std::string topic;
  std::string mtype;
  double p_i;
  double d_i;
  std::string level;
  std::mutex * p_mutex;

  TildePathConfig(uint32_t index, std::mutex * mtx)
  : index(index), p_mutex(mtx)
  {
    std::string fs = fmt::format("[{}]:{} >>> constructor({}) >>>", __func__, __LINE__, this->index);
    std::cout << fs.c_str() << std::endl;
    status = e_stat::ST_NONE;
    cur_j = 0l;
    completed_j = -1l;
    deadline_timer_manage = 0lu;
    r_i_j_1 = r_i_j = 0.0;
  }
  TildePathConfig(const TildePathConfig & c) = delete;
  ~TildePathConfig()
  {
    std::string fs = fmt::format("[{}]:{} <<< destructor({}) <<<", __func__, __LINE__, this->index);
    std::cout << fs.c_str() << std::endl;
    delete p_mutex;
  }

  // variables
  e_stat status;
  int64_t cur_j;
  int64_t completed_j;
  double periodic_timer_val;
  rclcpp::TimerBase::SharedPtr periodic_timer;
  rclcpp::TimerBase::SharedPtr interval_timer;
  DeadlineTimerMap deadline_timer;
  builtin_interfaces::msg::Time r_i_j_1_stamp;
  double r_i_j_1;
  double r_i_j;
  uint64_t deadline_timer_manage;
  uint64_t violation_count_thresh;
};

class TildeTimingMonitor : public rclcpp::Node
{
public:
  TildeTimingMonitor();
  bool get_debug_param() { return params_.debug_ctrl; }
  void registerNodeToDebug(const std::shared_ptr<TildeTimingMonitor> & node);
  double get_now();

private:
  struct Parameters
  {
    bool debug_ctrl;
  };
  Parameters params_{};

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  void loadTargetPaths();
  // Subscriber
  void onGenTopic(const std::shared_ptr<rclcpp::SerializedMessage> msg, std::shared_ptr<TildePathConfig> pinfo_ptr);
  void topicCallback(
    TildePathConfig & pinfo, double & pub_time, double & cur_ros, double & response_time);
  bool isOverDeadline(
    TildePathConfig & pinfo, double & pub_time, double & cur_ros, double & response_time);
  void checkExistingTimers(TildePathConfig & pinfo);
  void restartTimers(TildePathConfig & pinfo, double & cur_ros);
  // Timer
  void onIntervalTimer(TildePathConfig & pinfo);
  void onPeriodicTimer(TildePathConfig & pinfo);
  void onDeadlineTimer(TildePathConfig & pinfo, DeadlineTimer & dm);
  void startIntervalTimer(TildePathConfig & pinfo, double & time_val);
  void startPeriodicTimer(TildePathConfig & pinfo, double & time_val);
  void startDeadlineTimer(TildePathConfig & pinfo, double & start_time, double & time_val);
  bool isValidDeadlineTimer(TildePathConfig & pinfo, DeadlineTimer & dm);
  // Publisher
  rclcpp::Publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorDeadlineMiss>::SharedPtr
    pub_tilde_deadline_miss_;
  void pubDeadlineMiss(TildePathConfig & pinfo, int64_t & self_j, double & start);

  void stopDetect(TildePathConfig & pinfo);
};

}  // namespace tilde_timing_monitor
#endif  // TILDE_TIMING_MONITOR__TILDE_TIMING_MONITOR_CORE_HPP_
