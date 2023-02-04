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

#ifndef TILDE_TIMING_MONITOR__TILDE_TIMING_MONITOR_DEBUG_HPP_
#define TILDE_TIMING_MONITOR__TILDE_TIMING_MONITOR_DEBUG_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "tilde_msg/msg/message_tracking_tag.hpp"
#include "tilde_timing_monitor_interfaces/msg/tilde_timing_monitor_command.hpp"
#include "tilde_timing_monitor_interfaces/msg/tilde_timing_monitor_deadline_miss.hpp"
#include "tilde_timing_monitor_interfaces/msg/tilde_timing_monitor_infos.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include <cfloat>
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
class MinMax
{
public:
  MinMax()
  {
    mMax = mAccum = 0.0;
    mMin = DBL_MAX;
    mCount = 0;
  }
  void addData(double data)
  {
    if (data <= 0.0) {
      return;
    }
    mMin = std::min(mMin, data);
    mMax = std::max(mMax, data);
    mAccum += data;
    mCount++;
  }
  void addData(double data, double limit)
  {
    if (data <= 0.0) {
      return;
    }
    mMin = std::min(mMin, data);
    mMax = std::max(mMax, data);
    mAccum += data;
    mCount++;
    if (data >= limit) {
      over_count++;
      over_per_limit += data / limit;
    }
  }
  double getMin() { return (mCount == 0) ? 0 : mMin; }
  double getMax() { return mMax; }
  double getAve() { return mAccum / std::max((uint64_t)1, mCount); }
  uint64_t getCnt() { return mCount; }
  std::string getName() { return mName; }
  void setName(const char * name)
  {
    mName = name;
    mMin = DBL_MAX;
  }
  uint64_t getOver() { return over_count; }
  uint64_t getPerLimit() { return over_per_limit; }

protected:
  std::string mName;
  double mMin, mMax, mAccum;
  std::chrono::system_clock::time_point mPrev;
  uint64_t mCount;
  uint64_t over_count;
  uint64_t over_per_limit;
};

class RateMinMax : public MinMax
{
public:
  void addRate(double & pub_time, double & limit)
  {
    if (mPrevPub == 0.0) {
      mPrevPub = pub_time;
      return;
    }
    auto elapse = pub_time - mPrevPub;
    addData(elapse);
    mPrevPub = pub_time;
    if (elapse >= limit) {
      over_count++;
      over_per_limit += elapse / limit;
    }
  }
  void setName(const char * name)
  {
    mName = name;
    mMin = DBL_MAX;
    over_count = 0;
    over_per_limit = 0;
  }
  void setPrev(double & pub_time) { mPrevPub = pub_time; }

private:
  double mPrevPub;
};

class ElapseMinMax : public MinMax
{
public:
  void addElapse()
  {
    auto cur = std::chrono::system_clock::now();
    auto elapse_duration =
      std::chrono::duration_cast<std::chrono::nanoseconds>(cur - mPrev).count();
    auto elapse = static_cast<double>(elapse_duration);
    elapse /= (1000 * 1000 * 1000);
    addData(elapse);
  }
  void setName(const char * name)
  {
    mName = name;
    mMin = DBL_MAX;
  }
  void setPrev() { mPrev = std::chrono::system_clock::now(); }

private:
  std::chrono::system_clock::time_point mPrev;
};

using CbstatisMap = std::unordered_map<std::string, ElapseMinMax>;

class TildePathConfig;
class TildePathDebug
{
public:
  TildePathDebug(const TildePathConfig * pinfo_ptr) : pinfo_ptr(pinfo_ptr)
  {
    printf("--- [%s]:%04d called\n", __FILE__, __LINE__);
    response_time.setName("response_time");
    too_long_response_time.setName("too_long_response_time");
    hz.setName("hz");
    sub_interval.setName("sub_interval");
    com_delay.setName("com_delay");
  }
  const TildePathConfig * pinfo_ptr;
  //
  MinMax response_time;
  MinMax too_long_response_time;
  RateMinMax hz;
  RateMinMax sub_interval;
  MinMax com_delay;
};

using PathDebugInfoMap = std::map<uint32_t, const std::shared_ptr<TildePathDebug>>;

class TildeTimingMonitorDebug
{
public:
  TildeTimingMonitorDebug() {}
  TildeTimingMonitorDebug(TildeTimingMonitor * node, const char * version, bool debug_ctrl);

  // node
  std::shared_ptr<TildeTimingMonitor> node;
  const char * version;
  bool debug_ctrl;

  std::mutex tm_mutex_;
  // Publisher
  rclcpp::Publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorInfos>::SharedPtr
    pub_tm_statistics_;

  void registerPathDebugInfo(uint32_t key, std::shared_ptr<TildePathDebug> dinfo_ptr);
  bool topicStatis(TildePathConfig & pinfo, double & cur_ros, double & response_time);
  void log(std::string fs);
  void cbStatisEnter(const char * func);
  void cbStatisExit(const char * func);

private:
  // subscriber
  rclcpp::Subscription<
    tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand>::ConstSharedPtr cmd_sub_;

  // for statistics and debug
  void onCommand(
    const tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand::ConstSharedPtr msg);
  void pubCmdReqInfo();
  void cmdShowStatis();

  // others (debug)
  void printLog();
  void enLog(bool ope);
  void dispLogCtrl(bool ope);
};

}  // namespace tilde_timing_monitor
#endif  // TILDE_TIMING_MONITOR__TILDE_TIMING_MONITOR_DEBUG_HPP_
