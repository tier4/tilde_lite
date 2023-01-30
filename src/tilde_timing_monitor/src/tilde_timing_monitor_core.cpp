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
static const char * version = "v0.02";
static const char * mtt_topic = "message_tracking_tag";
static const char * tm_command_topic = "tilde_timing_monitor_command";
static const char * SHOW_INFO = "show info";
static const char * REQ_INFO = "req info";
static const char * PRINT_LOG = "show hist";
static const char * ENA_LOG = "histon";
static const char * DIS_LOG = "histoff";
static const char * DISP_ON = "dispon";
static const char * DISP_OFF = "dispoff";
static const char * PSEUDO_ROS_TIME_ON = "proson";
static const char * PSEUDO_ROS_TIME_OFF = "prosoff";

double init_pseudo_ros_time;
double init_dur_pseudo_ros_time;

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
std::vector<rclcpp::TimerBase::SharedPtr> periodic_timer_buffer_;
rclcpp::Subscription<
  tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand>::ConstSharedPtr cmd_sub_;
CbstatisMap cb_statis_map_;

TildeTimingMonitor::TildeTimingMonitor()
: Node(
    "tilde_timing_monitor", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  // Parameters
  get_parameter_or<bool>("statistics", params_.statistics, true);
  get_parameter_or<bool>("pseudo_ros_time", params_.pseudo_ros_time, false);
  get_parameter_or<std::string>("mode", params_.mode, "test");
  RCLCPP_INFO(
    get_logger(), "mode=%s statistics=%d pseudo_ros_time=%d", params_.mode.c_str(),
    params_.statistics, params_.pseudo_ros_time);

  // load topics and paths
  loadRequiredPaths(params_.mode);

  RCLCPP_INFO(get_logger(), "[%s]:%04d called", __func__, __LINE__);
  clock_.reset(new rclcpp::Clock(RCL_ROS_TIME));
  rclcpp::QoS qos = rclcpp::QoS{1};
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
  }

  // Publisher
  pub_tilde_deadline_miss_ =
    create_publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorDeadlineMiss>(
      "~/output/tilde_timing_monitor/deadline_miss", rclcpp::QoS{1});
  pub_tm_statistics_ =
    create_publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorInfos>(
      "~/output/tilde_timing_monitor/statistics", rclcpp::QoS{1});

  // command topic
  cmd_sub_ = create_subscription<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand>(
    tm_command_topic, qos,
    [this](tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand::ConstSharedPtr msg) {
      TildeTimingMonitor::onCommand(msg);
    });

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
      continue;  // Skip duprecated path
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

// all topic process
void TildeTimingMonitor::topicCallback(TildePathConfig & pinfo, double & cur_ros, double & response_time)
{
  bool over_f = topicStatis(pinfo, pinfo.r_i_j_1, cur_ros);
  pinfo.com_delay.addData(cur_ros - pinfo.r_i_j_1, pinfo.d_i);
  if(cur_ros - pinfo.r_i_j_1 > pinfo.d_i) {
    over_f = true; 
  }
  if(response_time > pinfo.d_i) {
    tooLongRespTimeStatis(pinfo, response_time);
    over_f = true;
  }
  if (over_f == true) {
    log(fmt::format(
      "--[{}]:{} <{}> DEADLINE OVER r_i_j_1={} {}", __func__, __LINE__, pinfo.path_name.c_str(),
      pinfo.r_i_j_1, pinfo.topic.c_str()));
    log(fmt::format(
      "--[{}]:{} <{}> L{} OK={} NG={} cur_j={} completed_j={}", __func__, __LINE__,
      pinfo.path_name.c_str(), pinfo.recv_count, pinfo.OK, pinfo.NG, pinfo.cur_j,
      pinfo.completed_j));
    return;
  }
  pinfo.r_i_j = pinfo.r_i_j_1 + pinfo.p_i;
  bool first = true;
  for (auto & kv : pinfo.deadline_timer) {
    auto & dm = kv.second;
    if (first) {
      respTimeStatis(pinfo, response_time);
      pinfo.completed_count++;
      pinfo.OK++;
      pinfo.completed_j = std::max(dm.self_j, pinfo.completed_j);
      pinfo.cur_j = pinfo.completed_j + 1;
      first = false;
      log(fmt::format(
        "--[{}]:{} <{}> dm[{}] {} cur_j={} completed_j={} start={:.6f} r_i_j_1={:.6f}", __func__,
        __LINE__, pinfo.path_name.c_str(), dm.self_j, "CAN->OK", pinfo.cur_j, pinfo.completed_j,
        dm.start_time, pinfo.r_i_j_1));
    } else {
      log(fmt::format(
        "--[{}]:{} <{}> dm[{}] {} cur_j={} completed_j={} start={:.6f} r_i_j_1={:.6f}", __func__,
        __LINE__, pinfo.path_name.c_str(), dm.self_j, "INGNORE", pinfo.cur_j, pinfo.completed_j,
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
  //
  pinfo.periodic_timer_val = pinfo.p_i;
  auto next_periodic_start = pinfo.r_i_j;
  log(fmt::format(
    "--[{}]:{} <{}> L{} OK={} NG={} cur_j={} completed_j={}", __func__, __LINE__,
    pinfo.path_name.c_str(), pinfo.recv_count, pinfo.OK, pinfo.NG, pinfo.cur_j, pinfo.completed_j));
  for (; next_periodic_start <= cur_ros; next_periodic_start += pinfo.p_i) {
    if ((next_periodic_start + pinfo.d_i) > cur_ros) {
      startDeadlineTimer(pinfo, next_periodic_start, (next_periodic_start + pinfo.d_i) - cur_ros);
    } else {
      pubDeadlineMiss(pinfo, pinfo.completed_j, next_periodic_start, true);
      pinfo.completed_j++;
      pinfo.presumed_deadline_miss_count++;
      pinfo.NG++;
    }
    pinfo.cur_j++;
  }
  if (next_periodic_start > cur_ros) {
    pinfo.periodic_timer_val = next_periodic_start - cur_ros;
  } else {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d ## perioic start error ros_cur=%lf next_periodic_start=%lf",
      __func__, __LINE__, cur_ros, next_periodic_start);
  }
  startIntervalTimer(pinfo, pinfo.periodic_timer_val);
  log(fmt::format(
    "--[{}]:{} <{}> L{} OK={} NG={} PERIODIC={:.6f} cur_j={} completed_j={}", __func__, __LINE__,
    pinfo.path_name.c_str(), pinfo.recv_count, pinfo.OK, pinfo.NG, pinfo.periodic_timer_val,
    pinfo.cur_j, pinfo.completed_j));
}

// mtt topic
void TildeTimingMonitor::onMttTopic(
  const MessageTrackingTag::ConstSharedPtr msg, TildePathConfig & pinfo)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);
  if (pinfo.status == e_stat::ST_NONE) {
    pinfo.status = e_stat::ST_INIT;
  }
  cbStatisEnter(__func__);

  double cur_ros = get_now();
  double pub_time = stamp_to_sec(msg->header.stamp);
  // double pub_time = stamp_to_sec(msg->output_info.header_stamp);
  pinfo.r_i_j_1_stamp = msg->input_infos[0].header_stamp;
  pinfo.r_i_j_1 = stamp_to_sec(pinfo.r_i_j_1_stamp);
  pinfo.recv_count += 1;
  log(fmt::format("|{:.6f}|[{}]:{} <{}> {:.6f} L{} OK={} NG={} {} dead={}", 
                   cur_ros , __func__, __LINE__, pinfo.path_name.c_str(), pinfo.r_i_j_1, pinfo.recv_count, pinfo.OK, pinfo.NG, pinfo.path_name.c_str(), pinfo.deadline_timer.size()));
  auto response_time = pub_time - pinfo.r_i_j_1;
  topicCallback(pinfo, cur_ros, response_time);

  cbStatisExit(__func__);
}

// general topic
void TildeTimingMonitor::onGenTopic(
  const std::shared_ptr<rclcpp::SerializedMessage> msg, TildePathConfig & pinfo)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);
  if (pinfo.status == e_stat::ST_NONE) {
    pinfo.status = e_stat::ST_INIT;
  }
  cbStatisEnter(__func__);

  std_msgs::msg::Header header_msg;
  auto serializer = rclcpp::Serialization<std_msgs::msg::Header>();
  serializer.deserialize_message(msg.get(), &header_msg);

  double cur_ros = get_now();
  double pub_time = stamp_to_sec(header_msg.stamp);
  pinfo.r_i_j_1 = pub_time;
  pinfo.recv_count += 1;
  log(fmt::format("|{:.6f}|[{}]:{} <{}> {:.6f} L{} OK={} NG={} {} dead={}", 
                   cur_ros , __func__, __LINE__, pinfo.path_name.c_str(), pinfo.r_i_j_1, pinfo.recv_count, pinfo.OK, pinfo.NG, pinfo.path_name.c_str(), pinfo.deadline_timer.size()));
  // Response time can be calculated for MTT, but for normal topics, it will be the time to subscribe
  auto response_time = cur_ros - pinfo.r_i_j_1;
  topicCallback(pinfo, cur_ros, response_time);

  cbStatisExit(__func__);
}

// Timer
void TildeTimingMonitor::startIntervalTimer(TildePathConfig & pinfo, double time_val)
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
  log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> {:.6f} INTERVAL start cur_j={} completed_j={}", get_now(), __func__,
    __LINE__, pinfo.path_name.c_str(), time_val, pinfo.cur_j, pinfo.completed_j));
}

void TildeTimingMonitor::startPeriodicTimer(TildePathConfig & pinfo, double time_val)
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
  log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> {:.6f} PERIODIC start cur_j={} completed_j={}", get_now(), __func__,
    __LINE__, pinfo.path_name.c_str(), time_val, pinfo.cur_j, pinfo.completed_j));
}

void TildeTimingMonitor::startDeadlineTimer(
  TildePathConfig & pinfo, double start_time, double time_val)
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
  log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> dm[{}] {:.6f} DEADLINE start cur_j={} completed_j={}", get_now(),
    __func__, __LINE__, pinfo.path_name.c_str(), dm.self_j, start_time, pinfo.cur_j,
    pinfo.completed_j));
}

// periodic timer
void TildeTimingMonitor::onIntervalTimer(TildePathConfig & pinfo)
{
  pinfo.interval_timer.reset();
  startPeriodicTimer(pinfo, pinfo.p_i);
  startDeadlineTimer(pinfo, get_now(), pinfo.d_i);
  pinfo.cur_j++;
}

void TildeTimingMonitor::onPeriodicTimer(TildePathConfig & pinfo)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);

  if (pinfo.status == e_stat::ST_NONE) return;
  cbStatisEnter(__func__);

  // periodic timer proc
  auto cur_ros = get_now();
  log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> periodic timer TO p_i={}", cur_ros, __func__, __LINE__,
    pinfo.path_name.c_str(), pinfo.p_i));
  // deadline timer proc
  startDeadlineTimer(pinfo, cur_ros, pinfo.d_i);
  pinfo.cur_j++;
  pinfo.status = e_stat::ST_DETECT;

  cbStatisExit(__func__);
}

// deadline timer
void TildeTimingMonitor::onDeadlineTimer(TildePathConfig & pinfo, DeadlineTimer dm)
{
  std::lock_guard<std::mutex> lock(*pinfo.tm_mutex);
  if (pinfo.status == e_stat::ST_NONE) return;
  if (pinfo.deadline_timer.find(dm.uniq) == pinfo.deadline_timer.end()) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## dm not found in deadline timer map", __func__, __LINE__,
      pinfo.path_name.c_str());
    return;
  }
  log(fmt::format(
    "|{:.6f}|[{}]:{} <{}> dm[{}] {:.6f} DEADLINE TIMEOUT cur_j={} completed_j={}", get_now(),
    __func__, __LINE__, pinfo.path_name.c_str(), dm.self_j, dm.start_time, pinfo.cur_j,
    pinfo.completed_j));
  if (pinfo.deadline_timer.empty()) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer empty", __func__, __LINE__,
      pinfo.path_name.c_str());
    return;
  }
  if (dm.valid == false) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer already canceled j=%lu [%lu]", __func__,
      __LINE__, pinfo.path_name.c_str(), dm.self_j, dm.uniq);
    pinfo.deadline_timer.erase(dm.uniq);
    return;
  }
  if (dm.timer.get() == nullptr) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer null", __func__, __LINE__,
      pinfo.path_name.c_str());
    return;
  }
  if (dm.self_j > pinfo.cur_j) {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer too old", __func__, __LINE__,
      pinfo.path_name.c_str());
    return;
  }
  cbStatisEnter(__func__);

  // deadline timer proc
  if (dm.self_j > pinfo.completed_j) {
    pinfo.completed_j = dm.self_j;
    pinfo.deadline_miss_count++;
    pinfo.NG++;
    pubDeadlineMiss(pinfo, dm.self_j, dm.start_time, false);
  } else {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d <%s> ## deadline timer illegal 'j' j=%lu completed_j=%lu", __func__,
      __LINE__, pinfo.path_name.c_str(), dm.self_j, pinfo.completed_j);
  }
  dm.timer->cancel();
  dm.timer.reset();
  dm.valid = false;
  pinfo.deadline_timer.erase(dm.uniq);

  cbStatisExit(__func__);
}

// publish
void TildeTimingMonitor::pubDeadlineMiss(
  TildePathConfig & pinfo, uint64_t self_j, double start, bool presumed)
{
  log(fmt::format(
    "[{}]:{} <{}> {:.6f} PUBLISH DEADLINE DETECT TOPIC cur_j={} completed_j={}", __func__, __LINE__,
    pinfo.path_name.c_str(), start, pinfo.cur_j, pinfo.completed_j));
  auto m = tilde_timing_monitor_interfaces::msg::TildeTimingMonitorDeadlineMiss();
  m.path_name = pinfo.path_name.c_str();
  m.topic = pinfo.topic.c_str();
  m.path_i = pinfo.path_i;
  m.presumed = presumed;
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

/*** for test & debug ***/
// command
void TildeTimingMonitor::cmdShowStatis()
{
  std::string fs = fmt::format("\n----- statistics ({}) -----", version);
  std::cout << fs.c_str() << std::endl;
  fs = fmt::format("mode={}", params_.mode);
  std::cout << fs.c_str() << std::endl;
  for (auto & pinfo : required_paths_map_.at(params_.mode)) {
    fs = fmt::format(
      "path_name={} path_i={} p_i={}(ms) d_i={}(ms)", pinfo.path_name.c_str(), pinfo.path_i,
      pinfo.p_i * 1000, pinfo.d_i * 1000);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("topic={} [{}]", pinfo.topic.c_str(), pinfo.mtype.c_str());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("path OK={} NG={}", pinfo.OK, pinfo.NG);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("path completed={}", pinfo.completed_count);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "deadline miss={} presumed miss={}", pinfo.deadline_miss_count,
      pinfo.presumed_deadline_miss_count);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("response time({}) min={} ave={} max={} (sec)  d_i over={} per limit={}",
                      pinfo.response_time.getCnt(), pinfo.response_time.getMin(), pinfo.response_time.getAve(), pinfo.response_time.getMax(), pinfo.response_time.getOver(), pinfo.response_time.getPerLimit());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "too long response time({}) min={} ave={} max={} (sec)",
      pinfo.too_long_response_time.getCnt(), pinfo.too_long_response_time.getMin(),
      pinfo.too_long_response_time.getAve(), pinfo.too_long_response_time.getMax());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("cur_j={} completed_j={}", pinfo.cur_j, pinfo.completed_j);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("r_i_j_1={} r_i_j={}", pinfo.r_i_j_1, pinfo.r_i_j);
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("topic({}) HZ min={} ave={} max={} (sec) d_i over={} per limit={}",
                      pinfo.hz.getCnt(), pinfo.hz.getMin(), pinfo.hz.getAve(), pinfo.hz.getMax(), pinfo.hz.getOver(), pinfo.hz.getPerLimit());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format(
      "topic({}) Sub interval min={} ave={} max={} (sec) d_i over={} per limit={}",
      pinfo.sub_interval.getCnt(), pinfo.sub_interval.getMin(), pinfo.sub_interval.getAve(),
      pinfo.sub_interval.getMax(), pinfo.sub_interval.getOver(), pinfo.sub_interval.getPerLimit());
    std::cout << fs.c_str() << std::endl;
    fs = fmt::format("commnication delay({}) min={} ave={} max={} (sec)  d_i over={} per limit={}",
                      pinfo.com_delay.getCnt(), pinfo.com_delay.getMin(), pinfo.com_delay.getAve(), pinfo.com_delay.getMax(), pinfo.com_delay.getOver(), pinfo.com_delay.getPerLimit());
    std::cout << fs.c_str() << std::endl;
    std::cout << "-- deadline timer ---" << std::endl;
    for (auto kv : pinfo.deadline_timer) {
      auto dm = kv.second;
      fs =
        fmt::format("-- j={}[{}] valid={} start={}", dm.self_j, dm.uniq, dm.valid, dm.start_time);
    }
    std::cout << "---------------------" << std::endl;
  }
  std::cout << "--- callbacks ---" << std::endl;
  for (auto & cb : cb_statis_map_) {
    fs = fmt::format(
      "[{}] ({}) min={} ave={} max={} (sec)", cb.first.c_str(), cb.second.getCnt(),
      cb.second.getMin(), cb.second.getAve(), cb.second.getMax());
    std::cout << fs.c_str() << std::endl;
  }
  std::cout << "(END)-----------------\n" << std::endl;
}

void TildeTimingMonitor::onCommand(
  const tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(tm_mutex_);
  cbStatisEnter(__func__);

  if (msg->command == SHOW_INFO) {
    cmdShowStatis();
  } else if (msg->command == REQ_INFO) {
    std::cout << "--- statistics & infos topic ---" << std::endl;
    pubCmdReqInfo();
  } else if (msg->command == PRINT_LOG) {
    std::cout << "--- start of log ---" << std::endl;
    printLog();
    std::cout << "--- end of log ---\n" << std::endl;
  } else if (msg->command == ENA_LOG) {
    std::cout << "--- logging enable ---" << std::endl;
    enLog(true);
  } else if (msg->command == DIS_LOG) {
    std::cout << "--- logging disable ---" << std::endl;
    enLog(false);
  } else if (msg->command == DISP_ON) {
    std::cout << "--- logging display ---" << std::endl;
    dispLogCtrl(true);
  } else if (msg->command == DISP_OFF) {
    std::cout << "--- logging display off ---" << std::endl;
    dispLogCtrl(false);
  } else if (msg->command == PSEUDO_ROS_TIME_ON) {
    std::cout << "--- pseudo ros time on ---" << std::endl;
    params_.pseudo_ros_time = true;
    pseudoRosTimeInit();
  } else if (msg->command == PSEUDO_ROS_TIME_OFF) {
    std::cout << "--- pseudo ros time off ---" << std::endl;
    params_.pseudo_ros_time = false;
  } else {
    RCLCPP_WARN(
      get_logger(), "[%s]:%04d ## not supported command [%s]", __func__, __LINE__,
      msg->command.c_str());
  }

  cbStatisExit(__func__);
}

// publish statistics
void TildeTimingMonitor::pubCmdReqInfo()
{
  // RCLCPP_INFO(get_logger(), "--[%s]:%04d called", __func__, __LINE__);
  auto m = tilde_timing_monitor_interfaces::msg::TildeTimingMonitorInfos();
  m.mode = params_.mode.c_str();
  for (auto & pinfo : required_paths_map_.at(params_.mode)) {
    auto p = tilde_timing_monitor_interfaces::msg::TildeTimingMonitorPathInfos();
    p.path_name = pinfo.path_name.c_str();
    p.topic = pinfo.topic;
    p.completed_count = pinfo.completed_count;
    p.presumed_deadline_miss_count = pinfo.presumed_deadline_miss_count;
    p.deadline_miss_count = pinfo.deadline_miss_count;
    p.response_count = pinfo.response_time.getCnt();
    p.response_time_min = pinfo.response_time.getMin();
    p.response_time_max = pinfo.response_time.getMax();
    p.response_time_ave = pinfo.response_time.getAve();
    p.too_long_response_count = pinfo.too_long_response_time.getCnt();
    p.too_long_response_time_min = pinfo.too_long_response_time.getMin();
    p.too_long_response_time_max = pinfo.too_long_response_time.getMax();
    p.response_time_ave = pinfo.response_time.getAve();
    p.path_i = pinfo.path_i;
    p.cur_j = pinfo.cur_j;
    p.completed_j = pinfo.completed_j;
    p.r_i_j_1_stamp = pinfo.r_i_j_1_stamp;
    p.r_i_j_1_float = pinfo.r_i_j_1;
    p.r_i_j_float = pinfo.r_i_j;
    p.recv_count = pinfo.hz.getCnt();
    p.hz_min = pinfo.hz.getMin();
    p.hz_max = pinfo.hz.getMax();
    p.hz_ave = pinfo.hz.getAve();
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
  m.header.stamp = get_clock()->now();
  pub_tm_statistics_->publish(m);
}

// statistics
void TildeTimingMonitor::respTimeStatis(TildePathConfig & pinfo, double & response_time)
{
  if(params_.statistics) {
    pinfo.response_time.addData(response_time, pinfo.d_i);
  }
}

void TildeTimingMonitor::tooLongRespTimeStatis(TildePathConfig & pinfo, double & response_time)
{
  if (params_.statistics) {
    pinfo.too_long_response_time.addData(response_time);
  }
}

double TildeTimingMonitor::get_now()
{
  if (!params_.pseudo_ros_time) {
    return nano_to_sec(get_clock()->now().nanoseconds());
  }
  return init_pseudo_ros_time +
         (nano_to_sec(steady_clock_->now().nanoseconds()) - init_dur_pseudo_ros_time);
}

bool TildeTimingMonitor::topicStatis(TildePathConfig & pinfo, double & s_stamp, double & cur_ros)
{
  if (params_.statistics) {
    // RCLCPP_INFO(get_logger(), "--[%s]:%04d called", __func__, __LINE__);
    auto hz_over = pinfo.hz.getOver();
    auto sub_over = pinfo.sub_interval.getOver();
    pinfo.hz.addRate(s_stamp, pinfo.d_i);
    pinfo.sub_interval.addRate(cur_ros, pinfo.d_i);
    if (hz_over != pinfo.hz.getOver() || sub_over != pinfo.sub_interval.getOver()) {
      return true;
    }
  }
  return false;
}

void TildeTimingMonitor::cbStatisEnter(const char * func)
{
  if (params_.statistics) {
    std::string fn = func;
    if (cb_statis_map_.find(fn) == cb_statis_map_.end()) {
      ElapseMinMax tmp;
      tmp.setName(func);
      cb_statis_map_[fn] = tmp;
    }
    auto & cs = cb_statis_map_[fn];
    cs.setPrev();
    // RCLCPP_INFO(get_logger(), ">>> %s [%s]", func, cs.getName().c_str());
  }
}

void TildeTimingMonitor::cbStatisExit(const char * func)
{
  if (params_.statistics) {
    std::string fn = func;
    auto & cs = cb_statis_map_[fn];
    cs.addElapse();
    // RCLCPP_INFO(get_logger(), "<<< %s [%s] ave=%lf cnt=%lu", func, cs.getName().c_str(),
    // cs.getAve(), cs.getCnt());
  }
}

static bool log_disp = false;
static bool enable_log = true;
std::deque<std::string> log_buffer_;
void TildeTimingMonitor::log(std::string fs)
{
  if (log_disp) {
    RCLCPP_INFO(get_logger(), fs.c_str());
  }
  if (!enable_log) return;
  double cur_ros = get_now();
  log_buffer_.push_back(fmt::format("[{:.6f}] {}", cur_ros, fs.c_str()));
  if (log_buffer_.size() >= 1000 * 100) {
    log_buffer_.erase(log_buffer_.begin(), log_buffer_.begin() + 1000);
  }
}
void TildeTimingMonitor::printLog()
{
  for (auto & fs : log_buffer_) {
    std::cout << fs.c_str() << std::endl;
  }
}
void TildeTimingMonitor::enLog(bool ope) { enable_log = ope; }
void TildeTimingMonitor::dispLogCtrl(bool ope) { log_disp = ope; }

void TildeTimingMonitor::ajustPseudoRosTime()
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
  ajustPseudoRosTime();
  RCLCPP_INFO(get_logger(), "%s:%04d\n", __func__, __LINE__);

  RCLCPP_INFO(
    get_logger(), "get_now()=%lf ros_time=%lf", get_now(),
    nano_to_sec(get_clock()->now().nanoseconds()));
  RCLCPP_INFO(
    get_logger(), "init_pseudo_ros_time=%lf init_dur_pseudo_ros_time=%lf", init_pseudo_ros_time,
    init_dur_pseudo_ros_time);
}

}  // namespace tilde_timing_monitor
