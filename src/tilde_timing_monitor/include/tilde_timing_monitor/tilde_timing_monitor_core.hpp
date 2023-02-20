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

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

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

enum class e_stat {
  ST_NONE,
  ST_INIT,
  ST_DETECT,
};

class TildePathConfig
{
public:
  uint32_t index;
  std::string path_name;
  uint64_t path_i;
  std::string topic;
  std::string mtype;
  double p_i;
  double d_i;
  std::string level;
  std::shared_ptr<std::mutex> tm_mutex;
  TildePathConfig()
  {
    std::shared_ptr<std::mutex> mtx(new std::mutex);
    status = e_stat::ST_NONE;
    cur_j = 0l;
    completed_j = -1l;
    tm_mutex = mtx;
    deadline_timer_manage = 0lu;
    deadline_miss_count = 0lu;
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
  // diagnostics
  uint64_t diag_threshold;
  uint64_t deadline_miss_count;
  uint64_t prev_deadline_miss_count;
};

using RequiredPaths = std::vector<TildePathConfig>;

struct KeyName
{
  static constexpr char autonomous_driving[] = "autonomous_driving";
  static constexpr char test[] = "test";
  static constexpr char test_sensing[] = "test_sensing";
};

class TildeTimingMonitor : public rclcpp::Node
{
public:
  TildeTimingMonitor();
  bool get_debug_param() { return params_.debug_ctrl; }
  bool get_ros_time_param() { return params_.pseudo_ros_time; }
  std::string get_mode_param() { return params_.mode; }

  void registerNodeToDebug(const std::shared_ptr<TildeTimingMonitor> & node);
  double get_now();

private:
  struct Parameters
  {
    bool debug_ctrl;
    bool pseudo_ros_time;
    std::string mode;
  };
  Parameters params_{};

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  std::map<std::string, RequiredPaths> required_paths_map_;

  void loadRequiredPaths(const std::string & key);

  // Subscriber
  void onGenTopic(const std::shared_ptr<rclcpp::SerializedMessage> msg, TildePathConfig & pinfo);
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

  void diagDataUpdate(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // debug
  void stopDetect(TildePathConfig & pinfo);

  void adjustPseudoRosTime();
  void pseudoRosTimeInit();
};

}  // namespace tilde_timing_monitor
#endif  // TILDE_TIMING_MONITOR__TILDE_TIMING_MONITOR_CORE_HPP_
