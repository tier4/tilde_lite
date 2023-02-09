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

#include "tilde_timing_monitor/tilde_timing_monitor_core.hpp"
#include "tilde_timing_monitor/tilde_timing_monitor_debug.hpp"

#include <algorithm>
#include <chrono>
#include <regex>
#include <string>
#include <vector>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace tilde_timing_monitor
{
// const
static constexpr char version[] = "v0.12";
static constexpr char mtt_topic[] = "message_tracking_tag";

double init_pseudo_ros_time;
double init_dur_pseudo_ros_time;

// utils
inline double nano_to_sec(double nano) { return nano / (1000.0 * 1000.0 * 1000.0); }
inline double stamp_to_sec(builtin_interfaces::msg::Time stamp)
{
  return stamp.sec + stamp.nanosec / (1000.0 * 1000.0 * 1000.0);
}
builtin_interfaces::msg::Time sec_to_stamp(double sec_time)
{
  // builtin_interfaces.msg.Time stamp;
  auto stamp = builtin_interfaces::msg::Time();
  auto sec = std::floor(sec_time);
  auto nano = (sec_time - sec) * (1000 * 1000 * 1000);
  stamp.sec = static_cast<uint32_t>(sec);
  stamp.nanosec = static_cast<uint32_t>(nano);
  return stamp;
}
std::vector<std::string> split(const std::string & str, const char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<rclcpp::Subscription<MessageTrackingTag>::SharedPtr> mtt_sub_buffer_;
std::vector<rclcpp::GenericSubscription::SharedPtr> gen_sub_buffer_;
std::shared_ptr<TildeTimingMonitorDebug> dbg_info_;

/**
 * tilde timing monitor node
 */
TildeTimingMonitor::TildeTimingMonitor()
: Node(
    "tilde_timing_monitor", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  // Parameters
  get_parameter_or<bool>("debug", params_.debug_ctrl, true);
  get_parameter_or<bool>("pseudo_ros_time", params_.pseudo_ros_time, false);
  get_parameter_or<std::string>("mode", params_.mode, "test");
  RCLCPP_INFO(
    get_logger(), "mode=%s debug=%d pseudo_ros_time=%d", params_.mode.c_str(), params_.debug_ctrl,
    params_.pseudo_ros_time);
  dbg_info_ = std::make_shared<TildeTimingMonitorDebug>(this, version, params_.debug_ctrl);

  // load topics and paths
  loadRequiredPaths(params_.mode);

  RCLCPP_INFO(get_logger(), "[%s]:%04d called", __func__, __LINE__);
  clock_.reset(new rclcpp::Clock(RCL_ROS_TIME));
  rclcpp::QoS qos = rclcpp::QoS{1};
  uint32_t index = 0;
  for (auto & pinfo : required_paths_map_.at(params_.mode)) {
    // Subscriber
    if (
      pinfo.topic.find(mtt_topic) != std::string::npos ||
      pinfo.topic.find("mtt") != std::string::npos) {
      const auto mtt_sub = create_subscription<MessageTrackingTag>(
        pinfo.topic, qos, [this, &pinfo](MessageTrackingTag::ConstSharedPtr msg) {
          TildeTimingMonitor::onMttTopic(msg, pinfo);
        });
      mtt_sub_buffer_.push_back(mtt_sub);
    } else {
      const auto gen_sub = create_generic_subscription(
        pinfo.topic, pinfo.mtype, qos,
        [this, &pinfo](const std::shared_ptr<rclcpp::SerializedMessage> msg) {
          TildeTimingMonitor::onGenTopic(msg, pinfo);
        });
      gen_sub_buffer_.push_back(gen_sub);
    }
    //
    dbg_info_->registerPathDebugInfo(index, std::make_shared<TildePathDebug>(&pinfo));
    pinfo.index = index++;
  }

  // Publisher
  pub_tilde_deadline_miss_ =
    create_publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorDeadlineMiss>(
      "~/output/tilde_timing_monitor/deadline_miss", rclcpp::QoS{1});

  // pseudo ros timer init
  if (params_.pseudo_ros_time) {
    pseudoRosTimeInit();
  }
  RCLCPP_INFO(get_logger(), "\n\n--- start ---\n");
}

// load required paths and params from config yaml
void TildeTimingMonitor::loadRequiredPaths(const std::string & key)
{
  const auto param_key = std::string("required_paths.") + key;

  const uint64_t depth = 4;
  const auto param_names = this->list_parameters({param_key}, depth).names;

  if (param_names.empty()) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  // Load path names from parameter key
  std::set<std::string> path_names;
  RequiredPaths required_paths;

  for (const auto & param_name : param_names) {
    // Example of param_name: required_paths.key.end_point_topic.parameter
    const auto split_names = split(param_name, '.');
    const auto & param_required_paths = split_names.at(0);
    const auto & param_key = split_names.at(1);
    const auto & param_topic = split_names.at(2);

    const auto & path_name_with_prefix =
      fmt::format("{0}.{1}.{2}", param_required_paths, param_key, param_topic);

    RCLCPP_INFO(
      get_logger(), "path_info: key=%s path=%s param=%s topic=%s", key.c_str(),
      param_required_paths.c_str(), param_key.c_str(), param_topic.c_str());

    if (path_names.count(path_name_with_prefix) != 0) {
      continue;  // Skip duplicated path
    }

    TildePathConfig path_config;
    // Register name
    path_names.insert(path_name_with_prefix);

    const auto path_name_key = path_name_with_prefix + std::string(".path_name");
    get_parameter_or(path_name_key, path_config.path_name, std::string("none"));
    path_config.topic = param_topic;
    const auto mtype_key = path_name_with_prefix + std::string(".mtype");
    get_parameter_or(
      mtype_key, path_config.mtype,
      std::string("tilde_timing_monitor_interfaces/msg/MessageTrackingTag"));
    const auto path_i_key = path_name_with_prefix + std::string(".path_i");
    get_parameter_or(path_i_key, path_config.path_i, 0lu);
    const auto periodic_key = path_name_with_prefix + std::string(".p_i");
    get_parameter_or(periodic_key, path_config.p_i, 0.0);
    path_config.p_i /= 1000;
    const auto deadline_key = path_name_with_prefix + std::string(".d_i");
    get_parameter_or(deadline_key, path_config.d_i, 0.0);
    path_config.d_i /= 1000;
    const auto level_key = path_name_with_prefix + std::string(".level");
    get_parameter_or(level_key, path_config.level, std::string("none"));

    RCLCPP_INFO(
      get_logger(), "path_name=%s %s [%s]\npath_i=%lu p_i=%lf d_i=%lf lv=%s",
      path_config.path_name.c_str(), path_config.topic.c_str(), path_config.mtype.c_str(),
      path_config.path_i, path_config.p_i, path_config.d_i, path_config.level.c_str());
    // Register each path
    required_paths.push_back(path_config);
  }
  required_paths_map_.insert(std::make_pair(key, required_paths));
}

/**
 * topic callback
 */
// mtt topic callback
void TildeTimingMonitor::onMttTopic(
  const MessageTrackingTag::ConstSharedPtr msg, TildePathConfig & pinfo)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);
  if (pinfo.status == e_stat::ST_NONE) {
    pinfo.status = e_stat::ST_INIT;
  }
  dbg_info_->cbStatisEnter(__func__);

  /* don't use SubTopicInfo
  double cur_ros = get_now();
  double pub_time = stamp_to_sec(msg->header.stamp);
  // double pub_time = stamp_to_sec(msg->output_info.header_stamp);
  pinfo.r_i_j_1_stamp = msg->input_infos[0].header_stamp;
  pinfo.r_i_j_1 = stamp_to_sec(pinfo.r_i_j_1_stamp);
  auto response_time = pub_time - pinfo.r_i_j_1;
  topicCallback(pinfo, pub_time, cur_ros, response_time);
  */

  double cur_ros = get_now();
  double pub_time = cur_ros;
  pinfo.r_i_j_1_stamp = msg->header.stamp;
  pinfo.r_i_j_1 = stamp_to_sec(pinfo.r_i_j_1_stamp);
  auto response_time = cur_ros - pinfo.r_i_j_1;
  topicCallback(pinfo, pub_time, cur_ros, response_time);

  dbg_info_->cbStatisExit(__func__);
}

// general topic callback
void TildeTimingMonitor::onGenTopic(
  const std::shared_ptr<rclcpp::SerializedMessage> msg, TildePathConfig & pinfo)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);
  if (pinfo.status == e_stat::ST_NONE) {
    pinfo.status = e_stat::ST_INIT;
  }
  dbg_info_->cbStatisEnter(__func__);

  std_msgs::msg::Header header_msg;
  try {
    auto serializer = rclcpp::Serialization<std_msgs::msg::Header>();
    serializer.deserialize_message(msg.get(), &header_msg);
  } catch (const rclcpp::exceptions::RCLError & e) {
    RCLCPP_INFO(get_logger(), "%s", e.what());
    exit(-1);
  }
  double cur_ros = get_now();
  double pub_time = cur_ros;
  pinfo.r_i_j_1_stamp = header_msg.stamp;
  pinfo.r_i_j_1 = stamp_to_sec(pinfo.r_i_j_1_stamp);
  // Response time can be calculated for MTT, but for normal topics, it will be the time to
  // subscribe
  auto response_time = cur_ros - pinfo.r_i_j_1;
  topicCallback(pinfo, pub_time, cur_ros, response_time);

  dbg_info_->cbStatisExit(__func__);
}

// all topic process
void TildeTimingMonitor::topicCallback(
  TildePathConfig & pinfo, double & pub_time, double & cur_ros, double & response_time)
{
  dbg_info_->log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> {:.6f} L{} OK={} NG={} dead={}", cur_ros, __func__, __LINE__,
    pinfo.path_name.c_str(), pinfo.r_i_j_1, dbg_info_->getValidTopicCounter(pinfo),
    dbg_info_->getOkCounter(pinfo), dbg_info_->getNgCounter(pinfo), pinfo.deadline_timer.size()));

  if (isOverDeadline(pinfo, pub_time, cur_ros, response_time)) {
    dbg_info_->log(fmt::format(
      "--[{}]:{} <{}> L{} DEADLINE OVER OK={} NG={} cur_j={} completed_j={}", __func__, __LINE__,
      pinfo.path_name.c_str(), dbg_info_->getValidTopicCounter(pinfo),
      dbg_info_->getOkCounter(pinfo), dbg_info_->getNgCounter(pinfo), pinfo.cur_j,
      pinfo.completed_j));
    return;
  }
  checkExistingTimers(pinfo);
  restartTimers(pinfo, cur_ros);
}

// check the deadline over
bool TildeTimingMonitor::isOverDeadline(
  TildePathConfig & pinfo, double & pub_time, double & cur_ros, double & response_time)
{
  /**
  auto s = fmt::format("{} cur_ros={:.6f} pub_time={:.6f} r_i_j_1(hz)={:.6f}",
    pinfo.path_name.c_str(), cur_ros, pub_time, pinfo.r_i_j_1);
  std::cout << s.c_str() << std::endl;
  **/

  bool over_f = dbg_info_->topicStatis(pinfo, pub_time, cur_ros, response_time);
  if (cur_ros - pinfo.r_i_j_1 >= pinfo.d_i || response_time >= pinfo.d_i) {
    over_f |= true;
  }
  dbg_info_->setTopicCounter(pinfo, over_f);
  return over_f;
}

// check the existing all timers
void TildeTimingMonitor::checkExistingTimers(TildePathConfig & pinfo)
{
  pinfo.r_i_j = pinfo.r_i_j_1 + pinfo.p_i;
  bool first = true;
  for (auto & kv : pinfo.deadline_timer) {
    auto & dm = kv.second;
    if (first) {
      dbg_info_->setCompletedCounter(pinfo);
      pinfo.completed_j = std::max(dm.self_j, pinfo.completed_j);
      pinfo.cur_j = pinfo.completed_j + 1;
      first = false;
      dbg_info_->log(fmt::format(
        "--[{}]:{} <{}> dm[{}] {} cur_j={} completed_j={} start={:.6f} r_i_j_1={:.6f}", __func__,
        __LINE__, pinfo.path_name.c_str(), dm.self_j, "CAN->OK", pinfo.cur_j, pinfo.completed_j,
        dm.start_time, pinfo.r_i_j_1));
    } else {
      dbg_info_->log(fmt::format(
        "--[{}]:{} <{}> dm[{}] {} cur_j={} completed_j={} start={:.6f} r_i_j_1={:.6f}", __func__,
        __LINE__, pinfo.path_name.c_str(), dm.self_j, "IGNORE", pinfo.cur_j, pinfo.completed_j,
        dm.start_time, pinfo.r_i_j_1));
    }
    if (dm.timer.get() == nullptr) {
      RCLCPP_WARN(get_logger(), "[%s]:%04d ## deadline timer null", __func__, __LINE__);
    } else {
      dm.timer->cancel();
      dm.timer.reset();
      dm.valid = false;
    }
  }
  pinfo.deadline_timer.clear();
}

// calc timer and restart
void TildeTimingMonitor::restartTimers(TildePathConfig & pinfo, double & cur_ros)
{
  pinfo.periodic_timer_val = pinfo.p_i;
  auto next_periodic_start = pinfo.r_i_j;
  dbg_info_->log(fmt::format(
    "--[{}]:{} <{}> L{} OK={} NG={} cur_j={} completed_j={}", __func__, __LINE__,
    pinfo.path_name.c_str(), dbg_info_->getValidTopicCounter(pinfo), dbg_info_->getOkCounter(pinfo),
    dbg_info_->getNgCounter(pinfo), pinfo.cur_j, pinfo.completed_j));
  for (; next_periodic_start <= cur_ros; next_periodic_start += pinfo.p_i) {
    if ((next_periodic_start + pinfo.d_i) > cur_ros) {
      auto deadline_time = (next_periodic_start + pinfo.d_i) - cur_ros;
      startDeadlineTimer(pinfo, next_periodic_start, deadline_time);
    } else {
      // this process doesn't work
      RCLCPP_WARN(get_logger(), "[%s]:%04d ## do not enter this route", __func__, __LINE__);
      pubDeadlineMiss(pinfo, pinfo.completed_j, next_periodic_start);
      pinfo.completed_j++;
      dbg_info_->setPresumedDeadlineCounter(pinfo);
    }
    pinfo.cur_j++;
  }
  if (next_periodic_start > cur_ros) {
    pinfo.periodic_timer_val = next_periodic_start - cur_ros;
  } else {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d ## periodic start error ros_cur=%lf next_periodic_start=%lf",
      __func__, __LINE__, cur_ros, next_periodic_start);
  }
  startIntervalTimer(pinfo, pinfo.periodic_timer_val);
  dbg_info_->log(fmt::format(
    "--[{}]:{} <{}> L{} OK={} NG={} PERIODIC={:.6f} cur_j={} completed_j={}", __func__, __LINE__,
    pinfo.path_name.c_str(), dbg_info_->getValidTopicCounter(pinfo), dbg_info_->getOkCounter(pinfo),
    dbg_info_->getNgCounter(pinfo), pinfo.periodic_timer_val, pinfo.cur_j, pinfo.completed_j));
}

/**
 * timer callback
 */
// interval timer(pre-periodic timer) callback
void TildeTimingMonitor::onIntervalTimer(TildePathConfig & pinfo)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);

  dbg_info_->cbStatisEnter(__func__);

  pinfo.interval_timer.reset();
  startPeriodicTimer(pinfo, pinfo.p_i);
  auto cur = get_now();
  startDeadlineTimer(pinfo, cur, pinfo.d_i);
  pinfo.cur_j++;

  dbg_info_->cbStatisExit(__func__);
}

// periodic timer callback
void TildeTimingMonitor::onPeriodicTimer(TildePathConfig & pinfo)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);

  if (pinfo.status == e_stat::ST_NONE) {
    return;
  }
  dbg_info_->cbStatisEnter(__func__);

  // periodic timer proc
  auto cur_ros = get_now();
  dbg_info_->log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> periodic timer TO p_i={}", cur_ros, __func__, __LINE__,
    pinfo.path_name.c_str(), pinfo.p_i));
  // deadline timer proc
  startDeadlineTimer(pinfo, cur_ros, pinfo.d_i);
  pinfo.cur_j++;
  pinfo.status = e_stat::ST_DETECT;

  dbg_info_->cbStatisExit(__func__);
}

// deadline timer callback
void TildeTimingMonitor::onDeadlineTimer(TildePathConfig & pinfo, DeadlineTimer & dm)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);

  if (pinfo.status == e_stat::ST_NONE) {
    return;
  }
  if (!isValidDeadlineTimer(pinfo, dm)) {
    return;
  }
  dbg_info_->cbStatisEnter(__func__);

  dbg_info_->log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> dm[{}] {:.6f} DEADLINE TIMEOUT cur_j={} completed_j={}", get_now(),
    __func__, __LINE__, pinfo.path_name.c_str(), dm.self_j, dm.start_time, pinfo.cur_j,
    pinfo.completed_j));

  // deadline timer proc
  if (dm.self_j > pinfo.completed_j) {
    pinfo.completed_j = dm.self_j;
    dbg_info_->setDeadlineCounter(pinfo);
    pubDeadlineMiss(pinfo, dm.self_j, dm.start_time);
  } else {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer illegal 'j' j=%lu completed_j=%lu", __func__,
      __LINE__, pinfo.path_name.c_str(), dm.self_j, pinfo.completed_j);
  }
  dm.timer->cancel();
  dm.timer.reset();
  dm.valid = false;
  pinfo.deadline_timer.erase(dm.uniq);

  dbg_info_->cbStatisExit(__func__);
}

// start interval timer
void TildeTimingMonitor::startIntervalTimer(TildePathConfig & pinfo, double & time_val)
{
  if (pinfo.interval_timer) {
    pinfo.interval_timer->cancel();
    pinfo.interval_timer.reset();
  }
  const auto tval = rclcpp::Rate(1 / time_val).period();
  pinfo.interval_timer = rclcpp::create_timer(
    this, get_clock(), tval, [this, &pinfo]() { TildeTimingMonitor::onIntervalTimer(pinfo); });
  if (pinfo.interval_timer.get() == nullptr) {
    RCLCPP_ERROR(
      get_logger(), "[%s]:%04d <%s> ## interval timer null", __func__, __LINE__,
      pinfo.path_name.c_str());
  }
  dbg_info_->log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> {:.6f} INTERVAL start cur_j={} completed_j={}", get_now(), __func__,
    __LINE__, pinfo.path_name.c_str(), time_val, pinfo.cur_j, pinfo.completed_j));
}

// start periodic timer
void TildeTimingMonitor::startPeriodicTimer(TildePathConfig & pinfo, double & time_val)
{
  if (pinfo.periodic_timer) {
    pinfo.periodic_timer->cancel();
    pinfo.periodic_timer.reset();
  }
  const auto tval = rclcpp::Rate(1 / time_val).period();
  pinfo.periodic_timer = rclcpp::create_timer(
    this, get_clock(), tval, [this, &pinfo]() { TildeTimingMonitor::onPeriodicTimer(pinfo); });
  if (pinfo.periodic_timer.get() == nullptr) {
    RCLCPP_ERROR(
      get_logger(), "[%s]:%04d <%s> ## periodic timer null", __func__, __LINE__,
      pinfo.path_name.c_str());
  }
  dbg_info_->log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> {:.6f} PERIODIC start cur_j={} completed_j={}", get_now(), __func__,
    __LINE__, pinfo.path_name.c_str(), time_val, pinfo.cur_j, pinfo.completed_j));
}

// start deadline timer
void TildeTimingMonitor::startDeadlineTimer(
  TildePathConfig & pinfo, double & start_time, double & time_val)
{
  DeadlineTimer dm;
  dm.uniq = pinfo.deadline_timer_manage++;
  dm.self_j = pinfo.cur_j;
  dm.start_time = start_time;
  dm.timer_val = time_val;
  dm.valid = true;
  pinfo.deadline_timer.emplace_hint(pinfo.deadline_timer.end(), dm.uniq, dm);
  auto & idm = pinfo.deadline_timer[dm.uniq];
  const auto tval = rclcpp::Rate(1 / dm.timer_val).period();
  idm.timer = rclcpp::create_timer(this, get_clock(), tval, [this, &pinfo, &idm]() {
    TildeTimingMonitor::onDeadlineTimer(pinfo, idm);
  });
  if (idm.timer.get() == nullptr) {
    RCLCPP_ERROR(
      get_logger(), "[%s]:%04d <%s> ## deadline timer null", __func__, __LINE__,
      pinfo.path_name.c_str());
  }
  dbg_info_->log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> dm[{}] {:.6f} DEADLINE start cur_j={} completed_j={}", get_now(),
    __func__, __LINE__, pinfo.path_name.c_str(), dm.self_j, start_time, pinfo.cur_j,
    pinfo.completed_j));
}

// check valid deadline timer
bool TildeTimingMonitor::isValidDeadlineTimer(TildePathConfig & pinfo, DeadlineTimer & dm)
{
  if (pinfo.deadline_timer.find(dm.uniq) == pinfo.deadline_timer.end()) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## dm not found in deadline timer map", __func__, __LINE__,
      pinfo.path_name.c_str());
    return false;
  }
  if (pinfo.deadline_timer.empty()) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer empty", __func__, __LINE__,
      pinfo.path_name.c_str());
    return false;
  }
  if (dm.valid == false) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer already canceled j=%lu [%lu]", __func__,
      __LINE__, pinfo.path_name.c_str(), dm.self_j, dm.uniq);
    pinfo.deadline_timer.erase(dm.uniq);
    return false;
  }
  if (dm.timer.get() == nullptr) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer null", __func__, __LINE__,
      pinfo.path_name.c_str());
    return false;
  }
  if (dm.self_j > pinfo.cur_j) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer too old", __func__, __LINE__,
      pinfo.path_name.c_str());
    return false;
  }
  return true;
}

// publish the deadline miss topic
void TildeTimingMonitor::pubDeadlineMiss(TildePathConfig & pinfo, int64_t & self_j, double & start)
{
  dbg_info_->log(fmt::format(
    "[{}]:{} <{}> {:.6f} PUBLISH DEADLINE DETECT TOPIC cur_j={} completed_j={}", __func__, __LINE__,
    pinfo.path_name.c_str(), start, pinfo.cur_j, pinfo.completed_j));
  auto m = tilde_timing_monitor_interfaces::msg::TildeTimingMonitorDeadlineMiss();
  m.path_name = pinfo.path_name.c_str();
  m.topic = pinfo.topic.c_str();
  m.path_i = pinfo.path_i;
  m.deadline_timer = pinfo.d_i;
  m.periodic_timer = pinfo.p_i;
  m.last_r_i_j_1_float = pinfo.r_i_j_1;
  m.last_r_i_j_1_stamp = sec_to_stamp(pinfo.r_i_j_1);
  m.deadline_timer_start = start;
  m.deadline_timer_start_stamp = sec_to_stamp(start);
  m.self_j = self_j;
  m.cur_j = pinfo.cur_j;
  m.completed_j = pinfo.completed_j;
  m.mode = params_.mode.c_str();
  m.header.stamp = get_clock()->now();
  pub_tilde_deadline_miss_->publish(m);
}

// utils
double TildeTimingMonitor::get_now()
{
  if (!params_.pseudo_ros_time) {
    return nano_to_sec(get_clock()->now().nanoseconds());
  }
  return init_pseudo_ros_time +
         (nano_to_sec(steady_clock_->now().nanoseconds()) - init_dur_pseudo_ros_time);
}

void TildeTimingMonitor::adjustPseudoRosTime()
{
  RCLCPP_INFO(get_logger(), "%s:%04d\n", __func__, __LINE__);
  for (int i = 0; i < 100; i++) {
    double pseudo = get_now();
    double ros_time = nano_to_sec(get_clock()->now().nanoseconds());
    if (pseudo == ros_time) {
      break;
    } else {
      init_pseudo_ros_time -= (pseudo - ros_time);
    }
  }
}

void TildeTimingMonitor::pseudoRosTimeInit()
{
  init_pseudo_ros_time = 0.0;
  for (;;) {
    init_pseudo_ros_time = nano_to_sec(get_clock()->now().nanoseconds());
    if (init_pseudo_ros_time != 0.0) {
      break;
    }
    sleep(1);
  }
  steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  init_dur_pseudo_ros_time = nano_to_sec(steady_clock_->now().nanoseconds());
  adjustPseudoRosTime();
  RCLCPP_INFO(get_logger(), "%s:%04d\n", __func__, __LINE__);

  RCLCPP_INFO(
    get_logger(), "get_now()=%lf ros_time=%lf", get_now(),
    nano_to_sec(get_clock()->now().nanoseconds()));
  RCLCPP_INFO(
    get_logger(), "init_pseudo_ros_time=%lf init_dur_pseudo_ros_time=%lf", init_pseudo_ros_time,
    init_dur_pseudo_ros_time);
}

}  // namespace tilde_timing_monitor
