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

#include "tilde_timing_monitor/tilde_timing_monitor_debug.hpp"

#include "tilde_timing_monitor/tilde_timing_monitor_core.hpp"

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace tilde_timing_monitor
{
// const
static constexpr char tm_command_topic[] = "tilde_timing_monitor_command";
static constexpr char SHOW_INFO[] = "show info";
static constexpr char REQ_INFO[] = "req info";
static constexpr char PRINT_LOG[] = "show hist";
static constexpr char CLR_INFO[] = "clrinfo";
static constexpr char ENA_DETECT[] = "deton";
static constexpr char DIS_DETECT[] = "detoff";
static constexpr char ENA_DBG[] = "dbgon";
static constexpr char DIS_DBG[] = "dbgoff";
static constexpr char ENA_LOG[] = "histon";
static constexpr char DIS_LOG[] = "histoff";
static constexpr char DISP_ON[] = "dispon";
static constexpr char DISP_OFF[] = "dispoff";

PathDebugInfoMap path_debug_info_;
CbstatisMap cb_statis_map_;

// register TildePathDebug
void TildeTimingMonitorDebug::registerPathDebugInfo(
  uint32_t key, const std::shared_ptr<TildePathDebug> dinfo_ptr)
{
  path_debug_info_.insert(std::make_pair(key, dinfo_ptr));
}

// TildeTimingMonitorDebug constructor
TildeTimingMonitorDebug::TildeTimingMonitorDebug(const char * version, bool debug_ctrl)
: version(version), debug_ctrl(debug_ctrl), enable_log(debug_ctrl)
{
  log_disp = false;
}

void TildeTimingMonitorDebug::registerNodeToDebug(const std::shared_ptr<TildeTimingMonitor> & node)
{
  this->node = node;
  rclcpp::QoS qos = rclcpp::QoS{1};
  // Publisher
  pub_tm_statistics_ =
    node->create_publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorInfos>(
      "~/output/tilde_timing_monitor/statistics", qos);
  // command topic
  cmd_sub_ =
    node->create_subscription<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand>(
      tm_command_topic, qos,
      [this](tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand::ConstSharedPtr msg) {
        TildeTimingMonitorDebug::onCommand(msg);
      });
  std::string fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
  RCLCPP_INFO(node->get_logger(), "\n\n--- %s ---\n", fs.c_str());
}

// show stats
void TildeTimingMonitorDebug::cmdShowStatis()
{
  std::string fs = fmt::format("\n----- statistics ({}) -----", this->version);
  std::cout << fs.c_str() << std::endl;
  fs = fmt::format("mode={}", this->node->get_mode_param());
  std::cout << fs.c_str() << std::endl;
  for (auto pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    auto pinfo_ptr = dinfo_ptr->pinfo_ptr;
    fs = fmt::format(
      "path_name={} path_i={} p_i={}(ms) d_i={}(ms)", pinfo_ptr->path_name.c_str(),
      pinfo_ptr->path_i, pinfo_ptr->p_i * 1000, pinfo_ptr->d_i * 1000);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("topic={} [{}]", pinfo_ptr->topic.c_str(), pinfo_ptr->mtype.c_str());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("deadline detect={}", (dinfo_ptr->enable_detect == true) ? "true" : "false");
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "topic valid={} discard={}", dinfo_ptr->valid_topic_count, dinfo_ptr->discard_topic_count);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "path OK={} NG={}", dinfo_ptr->completed_count,
      dinfo_ptr->deadline_miss_count + dinfo_ptr->false_deadline_miss_count +
        dinfo_ptr->presumed_deadline_miss_count);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("path completed={}", dinfo_ptr->completed_count);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "deadline miss={} false_miss={} presumed miss={}", dinfo_ptr->deadline_miss_count,
      dinfo_ptr->false_deadline_miss_count, dinfo_ptr->presumed_deadline_miss_count);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "response time({}) min={:.6f} ave={:.6f} max={:.6f} (sec)", dinfo_ptr->response_time.getCnt(),
      dinfo_ptr->response_time.getMin(), dinfo_ptr->response_time.getAve(),
      dinfo_ptr->response_time.getMax());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "too long response time({}) min={:.6f} ave={:.6f} max={:.6f} (sec)",
      dinfo_ptr->too_long_response_time.getCnt(), dinfo_ptr->too_long_response_time.getMin(),
      dinfo_ptr->too_long_response_time.getAve(), dinfo_ptr->too_long_response_time.getMax());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("cur_j={} completed_j={}", pinfo_ptr->cur_j, pinfo_ptr->completed_j);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("r_i_j_1={:.6f} r_i_j={:.6f}", pinfo_ptr->r_i_j_1, pinfo_ptr->r_i_j);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "topic({}) HZ min={:.6f} ave={:.6f} max={:.6f} (sec) d_i over={} per limit={}",
      dinfo_ptr->hz.getCnt(), dinfo_ptr->hz.getMin(), dinfo_ptr->hz.getAve(),
      dinfo_ptr->hz.getMax(), dinfo_ptr->hz.getOver(), dinfo_ptr->hz.getPerLimit());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "topic({}) Sub interval min={:.6f} ave={:.6f} max={:.6f} (sec) d_i over={} per limit={}",
      dinfo_ptr->sub_interval.getCnt(), dinfo_ptr->sub_interval.getMin(),
      dinfo_ptr->sub_interval.getAve(), dinfo_ptr->sub_interval.getMax(),
      dinfo_ptr->sub_interval.getOver(), dinfo_ptr->sub_interval.getPerLimit());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "communication delay({}) min={:.6f} ave={:.6f} max={:.6f} (sec)",
      dinfo_ptr->com_delay.getCnt(), dinfo_ptr->com_delay.getMin(), dinfo_ptr->com_delay.getAve(),
      dinfo_ptr->com_delay.getMax());
    std::cout << fs.c_str() << std::endl;
    std::cout << "-- deadline timer ---" << std::endl;
    for (auto kv : pinfo_ptr->deadline_timer) {
      auto dm = kv.second;
      fs = fmt::format(
        "-- j={}[{}] valid={} start={:.6f}", dm.self_j, dm.uniq, dm.valid, dm.start_time);
    }
    std::cout << "---------------------" << std::endl;
  }
  if (debug_ctrl) {
    std::cout << "--- callbacks ---" << std::endl;
    for (auto & cb : cb_statis_map_) {
      fs = fmt::format(
        "[{}] ({}) min={:.6f} ave={:.6f} max={:.6f} (sec)", cb.first.c_str(), cb.second.getCnt(),
        cb.second.getMin(), cb.second.getAve(), cb.second.getMax());
      std::cout << fs.c_str() << std::endl;
    }
  }
  std::cout << "(END)-----------------\n" << std::endl;
}

// command topic callback
void TildeTimingMonitorDebug::onCommand(
  const tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(tm_mutex_);
  cbStatisEnter(__func__);

  std::string fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
  if (msg->command == SHOW_INFO) {
    cmdShowStatis();
  } else if (msg->command == REQ_INFO) {
    std::cout << "--- statistics & infos topic ---" << std::endl;
    std::cout << fs.c_str() << std::endl;
    pubCmdReqInfo();
  } else if (msg->command == PRINT_LOG) {
    std::cout << "--- start of log ---" << std::endl;
    printLog();
    std::cout << "--- end of log ---\n" << std::endl;
    std::cout << fs.c_str() << std::endl;
  } else if (msg->command == CLR_INFO) {
    std::cout << "--- clear statics & infos ---" << std::endl;
    clearInfo();
  } else if (msg->command == ENA_DETECT) {
    std::cout << "--- restart detect ---" << std::endl;
    detectCtrl(true);
  } else if (msg->command == DIS_DETECT) {
    std::cout << "--- stop detect ---" << std::endl;
    detectCtrl(false);
  } else if (msg->command == ENA_LOG) {
    std::cout << "--- logging enable ---" << std::endl;
    enLog(true);
    fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
    std::cout << fs.c_str() << std::endl;
  } else if (msg->command == DIS_LOG) {
    std::cout << "--- logging disable ---" << std::endl;
    enLog(false);
    fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
    std::cout << fs.c_str() << std::endl;
  } else if (msg->command == ENA_DBG) {
    std::cout << "--- debug enable ---" << std::endl;
    enDbg(true);
    fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
    std::cout << fs.c_str() << std::endl;
  } else if (msg->command == DIS_DBG) {
    std::cout << "--- debug disable ---" << std::endl;
    enDbg(false);
    fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
    std::cout << fs.c_str() << std::endl;
  } else if (msg->command == DISP_ON) {
    std::cout << "--- logging display ---" << std::endl;
    dispLogCtrl(true);
    fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
    std::cout << fs.c_str() << std::endl;
  } else if (msg->command == DISP_OFF) {
    std::cout << "--- logging display off ---" << std::endl;
    dispLogCtrl(false);
    fs = fmt::format("debug={} log={} disp={}", debug_ctrl, enable_log, log_disp);
    std::cout << fs.c_str() << std::endl;
  } else {
    RCLCPP_WARN(
      this->node->get_logger(), "[%s]:%04d ## not supported command [%s]", __func__, __LINE__,
      msg->command.c_str());
  }

  cbStatisExit(__func__);
}

// publish statistics
void TildeTimingMonitorDebug::pubCmdReqInfo()
{
  if (!debug_ctrl) {
    return;
  }
  // RCLCPP_INFO(this->node->get_logger(), "--[%s]:%04d called", __func__, __LINE__);
  auto m = tilde_timing_monitor_interfaces::msg::TildeTimingMonitorInfos();
  m.mode = this->node->get_mode_param().c_str();
  for (auto & pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    auto pinfo_ptr = dinfo_ptr->pinfo_ptr;
    auto p = tilde_timing_monitor_interfaces::msg::TildeTimingMonitorPathInfos();
    p.path_name = pinfo_ptr->path_name.c_str();
    p.topic = pinfo_ptr->topic;
    p.valid_topic_count = dinfo_ptr->valid_topic_count;
    p.discard_topic_count = dinfo_ptr->discard_topic_count;
    p.completed_count = dinfo_ptr->completed_count;
    p.deadline_miss_count = dinfo_ptr->deadline_miss_count;
    p.false_deadline_miss_count = dinfo_ptr->false_deadline_miss_count;
    p.presumed_deadline_miss_count = dinfo_ptr->presumed_deadline_miss_count;
    p.response_count = dinfo_ptr->response_time.getCnt();
    p.response_time_min = dinfo_ptr->response_time.getMin();
    p.response_time_max = dinfo_ptr->response_time.getMax();
    p.response_time_ave = dinfo_ptr->response_time.getAve();
    p.too_long_response_count = dinfo_ptr->too_long_response_time.getCnt();
    p.too_long_response_time_min = dinfo_ptr->too_long_response_time.getMin();
    p.too_long_response_time_max = dinfo_ptr->too_long_response_time.getMax();
    p.too_long_response_time_ave = dinfo_ptr->too_long_response_time.getAve();
    p.path_i = pinfo_ptr->path_i;
    p.cur_j = pinfo_ptr->cur_j;
    p.completed_j = pinfo_ptr->completed_j;
    p.r_i_j_1_stamp = pinfo_ptr->r_i_j_1_stamp;
    p.r_i_j_1_float = pinfo_ptr->r_i_j_1;
    p.r_i_j_float = pinfo_ptr->r_i_j;
    p.hz_min = dinfo_ptr->hz.getMin();
    p.hz_max = dinfo_ptr->hz.getMax();
    p.hz_ave = dinfo_ptr->hz.getAve();
    p.sub_min = dinfo_ptr->sub_interval.getMin();
    p.sub_max = dinfo_ptr->sub_interval.getMax();
    p.sub_ave = dinfo_ptr->sub_interval.getAve();
    m.path_info.push_back(p);
  }
  for (auto & cb : cb_statis_map_) {
    auto c = tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCbLatency();
    c.cb_name = cb.first.c_str();
    c.cb_min = cb.second.getMin();
    c.cb_max = cb.second.getMax();
    c.cb_ave = cb.second.getAve();
    c.cb_count = cb.second.getCnt();
    m.cb_latency.push_back(c);
  }
  m.header.stamp = this->node->get_clock()->now();
  pub_tm_statistics_->publish(m);
}

// statistics
bool TildeTimingMonitorDebug::topicStatis(
  TildePathConfig & pinfo, double & pub_time, double & cur_ros, double & response_time)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  bool over_f = false;
  auto hz_over = dinfo_ptr->hz.getOver();
  auto sub_over = dinfo_ptr->sub_interval.getOver();
  dinfo_ptr->hz.addRate(pinfo.r_i_j_1, pinfo.d_i);
  dinfo_ptr->sub_interval.addRate(cur_ros, pinfo.d_i);
  if (hz_over != dinfo_ptr->hz.getOver() || sub_over != dinfo_ptr->sub_interval.getOver()) {
    over_f = true;
  }
  dinfo_ptr->com_delay.addData(cur_ros - pinfo.r_i_j_1, pinfo.d_i);
  dinfo_ptr->response_time.addData(response_time);
  if (response_time >= pinfo.d_i) {
    dinfo_ptr->too_long_response_time.addData(response_time);
    over_f = true;
  } else if (response_time + (cur_ros - pub_time) >= pinfo.d_i) {
    dinfo_ptr->false_deadline_miss_count++;
  }
  return over_f;
}

void TildeTimingMonitorDebug::setTopicCounter(TildePathConfig & pinfo, bool discard)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  if (discard) {
    dinfo_ptr->discard_topic_count++;
  } else {
    dinfo_ptr->valid_topic_count++;
  }
}

void TildeTimingMonitorDebug::setCompletedCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->completed_count++;
}

void TildeTimingMonitorDebug::setDeadlineCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->deadline_miss_count++;
}

void TildeTimingMonitorDebug::setFalseDeadlineCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->false_deadline_miss_count++;
}

void TildeTimingMonitorDebug::setPresumedDeadlineCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->presumed_deadline_miss_count++;
}

uint64_t TildeTimingMonitorDebug::getOkCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->completed_count;
}

uint64_t TildeTimingMonitorDebug::getNgCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->deadline_miss_count + dinfo_ptr->false_deadline_miss_count +
         dinfo_ptr->presumed_deadline_miss_count;
}

uint64_t TildeTimingMonitorDebug::getValidTopicCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->valid_topic_count;
}

uint64_t TildeTimingMonitorDebug::getDiscardTopicCounter(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->discard_topic_count;
}

// measurement callback process time
void TildeTimingMonitorDebug::cbStatisEnter(const char * func)
{
  if (debug_ctrl) {
    std::string fn = func;
    if (cb_statis_map_.find(fn) == cb_statis_map_.end()) {
      ElapseMinMax tmp;
      tmp.setName(func);
      cb_statis_map_[fn] = tmp;
    }
    auto & cs = cb_statis_map_[fn];
    cs.setPrev();
  }
}

void TildeTimingMonitorDebug::cbStatisExit(const char * func)
{
  if (debug_ctrl) {
    std::string fn = func;
    auto & cs = cb_statis_map_[fn];
    cs.addElapse();
  }
}

// logging
std::deque<std::string> log_buffer_;
void TildeTimingMonitorDebug::log(std::string fs)
{
  if (!enable_log) {
    return;
  }
  if (log_disp) {
    RCLCPP_INFO(this->node->get_logger(), fs.c_str());
  }
  double cur_ros = this->node->get_now();
  log_buffer_.push_back(fmt::format("[{:.6f}] {}", cur_ros, fs.c_str()));
  if (log_buffer_.size() >= 1000 * 100) {
    log_buffer_.erase(log_buffer_.begin(), log_buffer_.begin() + 1000);
  }
}

void TildeTimingMonitorDebug::printLog()
{
  if (!enable_log) {
    return;
  }
  for (auto & fs : log_buffer_) {
    std::cout << fs.c_str() << std::endl;
  }
}
void TildeTimingMonitorDebug::enDbg(bool ope)
{
  debug_ctrl = ope;
  enable_log = ope;
}

void TildeTimingMonitorDebug::enLog(bool ope) { enable_log = ope; }
void TildeTimingMonitorDebug::dispLogCtrl(bool ope) { log_disp = ope; }

void TildeTimingMonitorDebug::clearInfo()
{
  for (auto & pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    dinfo_ptr->completed_count = 0lu;
    dinfo_ptr->deadline_miss_count = 0lu;
    dinfo_ptr->false_deadline_miss_count = 0lu;
    dinfo_ptr->presumed_deadline_miss_count = 0lu;
    dinfo_ptr->valid_topic_count = 0lu;
    dinfo_ptr->discard_topic_count = 0lu;
    dinfo_ptr->response_time.setName("response_time");
    dinfo_ptr->too_long_response_time.setName("too_long_response_time");
    dinfo_ptr->hz.setName("hz");
    dinfo_ptr->sub_interval.setName("sub_interval");
    dinfo_ptr->com_delay.setName("com_delay");
  }
  cb_statis_map_.clear();
  log_buffer_.clear();
}

void TildeTimingMonitorDebug::detectCtrl(bool ope)
{
  for (auto & pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    dinfo_ptr->enable_detect = ope;
  }
}

bool TildeTimingMonitorDebug::getEnableDetect(TildePathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->enable_detect;
}

void TildeTimingMonitorDebug::setEnableDetect(TildePathConfig & pinfo, bool ope)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->enable_detect = ope;
}

}  // namespace tilde_timing_monitor
