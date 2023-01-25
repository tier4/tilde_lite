#ifndef TILDE_AGGREGATOR__TILDE_AGGREGATOR_CORE_HPP_
#define TILDE_AGGREGATOR__TILDE_AGGREGATOR_CORE_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "tilde_msg/msg/message_tracking_tag.hpp"
#include "tilde_timing_monitor_interfaces/msg/tilde_timing_monitor_deadline_miss.hpp"
#include "tilde_timing_monitor_interfaces/msg/tilde_timing_monitor_infos.hpp"
#include "tilde_timing_monitor_interfaces/msg/tilde_timing_monitor_command.hpp"

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>
#include <chrono>

namespace tilde_timing_monitor
{
class MinMax {
  public:
    MinMax() {
      mMax = mAccum = 0.0; 
      mMin = 10000000.0;
      mCount = 0;
    }
    void addData(double data) {
      if(data <= 0.0) return;
      mMin = std::min(mMin, data); 
      mMax = std::max(mMax, data); 
      mAccum += data; 
      mCount++;
    }
    void addData(double data, double limit) {
      if(data <= 0.0) return;
      mMin = std::min(mMin, data); 
      mMax = std::max(mMax, data); 
      mAccum += data; 
      mCount++;
      if(data >= limit) {
        over_count++;
        over_per_limit += data / limit;
      }
    }
    double getMin(){return (mCount == 0)? 0: mMin;}
    double getMax(){return mMax;}
    double getAve(){return mAccum/std::max((uint64_t)1,mCount);}
    uint64_t getCnt(){return mCount;}
    std::string getName(){return mName;}
    void setName(const char *name){
      mName = name;
      mMin = 10000000.0;
    }
    uint64_t getOver() {return over_count;}
    uint64_t getPerLimit() {return over_per_limit;}
  protected:
    std::string mName;
    double mMin, mMax, mAccum;
    std::chrono::system_clock::time_point mPrev;
    uint64_t mCount;
    uint64_t over_count;
    uint64_t over_per_limit;
};

class RateMinMax : public MinMax {
  public:
    void addRate(double & pub_time, double & limit) {
      if(mPrevPub == 0.0) {
        mPrevPub = pub_time;
        return;
      }
      auto elapse = pub_time - mPrevPub;
      addData(elapse);
      mPrevPub = pub_time;
      if(elapse >= limit) {
        over_count++;
        over_per_limit += elapse / limit;
      }
    }
    void setName(const char *name) {
      mName = name;
      mMin = 10000000.0;
      over_count = 0;
      over_per_limit = 0;
    }
    void setPrev(double & pub_time){mPrevPub = pub_time;}
  private:
    double mPrevPub;
};

class ElapseMinMax : public MinMax {
  public:
    void addElapse() {
      auto cur = std::chrono::system_clock::now();
      auto elapse_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(cur - mPrev).count();
      auto elapse = static_cast<double>(elapse_duration);
      elapse /= (1000*1000*1000);
      addData(elapse);
    }
    void setName(const char *name) {
      mName = name;
      mMin = 10000000.0;
    }
    void setPrev(){mPrev = std::chrono::system_clock::now();}
  private:
    std::chrono::system_clock::time_point mPrev;
};

using MessageTrackingTag = tilde_msg::msg::MessageTrackingTag;
using CbstatisMap = std::map<std::string, ElapseMinMax>;

struct DeadlineTimer {
  int64_t self_j;
  double start_time;
  double timer_val;
  rclcpp::TimerBase::SharedPtr timer;
  uint64_t uniq;
  bool valid;
};

using DeadlineTimerMap = std::map<uint64_t, DeadlineTimer>;

enum class e_stat {ST_NONE, ST_INIT, ST_DETECT,};
class TildePathConfig {
public:
  std::string path_name;
  uint64_t path_i;
  std::string topic;
  std::string mtype;
  double p_i;
  double d_i;
  std::string level;
  std::shared_ptr<std::mutex> tm_mutex;
  TildePathConfig() {
    std::shared_ptr<std::mutex> mtx(new std::mutex);
    status = e_stat::ST_NONE;
    cur_j = 0l;
    completed_j = -1l;
    completed_count = 0lu;
    deadline_miss_count = 0lu;
    presumed_deadline_miss_count = 0lu;
    tm_mutex = mtx;
    response_time.setName("response_time");
    too_long_response_time.setName("too_long_response_time");
    hz.setName("hz");
    sub_interval.setName("sub_interval");
    com_delay.setName("com_delay");
    deadline_timer_manage = 0lu;
    recv_count = 0lu;
    OK = 0lu;
    NG = 0lu;
  }
  // variables
  e_stat status;
  int64_t cur_j;
  int64_t completed_j;
  double prev_tick_time;
  double periodic_timer_val;
  rclcpp::TimerBase::SharedPtr periodic_timer;
  rclcpp::TimerBase::SharedPtr interval_timer;
  DeadlineTimerMap deadline_timer;
  builtin_interfaces::msg::Time r_i_j_1_stamp;
  double r_i_j_1;
  double r_i_j;
  uint64_t deadline_timer_manage;
  uint64_t recv_count;
  uint64_t OK;
  uint64_t NG;
  //
  MinMax response_time;
  MinMax too_long_response_time;
  uint64_t completed_count;
  uint64_t deadline_miss_count;
  uint64_t presumed_deadline_miss_count;
  RateMinMax hz;
  RateMinMax sub_interval;
  MinMax com_delay;
};

using RequiredPaths = std::vector<TildePathConfig>;

struct KeyName
{
  static constexpr const char * autonomous_driving = "autonomous_driving";
  static constexpr const char * test = "test";
  static constexpr const char * test_sensing = "test_sensing";
};

class TildeTimingMonitor : public rclcpp::Node
{
public:
  TildeTimingMonitor();

private:
  struct Parameters
  {
    bool statistics;
    bool pseudo_ros_time;
    std::string mode;
  };

  Parameters params_{};

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  std::unordered_map<std::string, RequiredPaths> required_paths_map_;

  std::mutex tm_mutex_;

  void loadRequiredPaths(const std::string & key);

  // Timer
  void onIntervalTimer(TildePathConfig & pinfo);
  void startIntervalTimer(TildePathConfig & pinfo, double time_val);
  void startPeriodicTimer(TildePathConfig & pinfo, double time_val);
  void onPeriodicTimer(TildePathConfig & pinfo);
  void onDeadlineTimer(TildePathConfig & pinfo, DeadlineTimer dm);
  void startDeadlineTimer(TildePathConfig & pinfo, double start_time, double time_val);
  double get_now();

  // Subscriber
  void onMttTopic(const MessageTrackingTag::ConstSharedPtr msg, TildePathConfig & pinfo);
  void onGenTopic(const std::shared_ptr<rclcpp::SerializedMessage> msg, TildePathConfig & pinfo);
  void topicCallback(TildePathConfig & pinfo, double & cur_ros, double & response_time);

  // Publisher
  rclcpp::Publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorDeadlineMiss>::SharedPtr pub_tilde_deadline_miss_;
  rclcpp::Publisher<tilde_timing_monitor_interfaces::msg::TildeTimingMonitorInfos>::SharedPtr pub_tm_statistics_;
  void pubDeadlineMiss(TildePathConfig & pinfo, uint64_t self_j, double start, bool presumed=false);

  // for statistics and debug
  void onCommand(const tilde_timing_monitor_interfaces::msg::TildeTimingMonitorCommand::ConstSharedPtr msg);
  void pubCmdReqInfo();
  void respTimeStatis(TildePathConfig & pinfo, double & response_time);
  void tooLongRespTimeStatis(TildePathConfig & pinfo, double & response_time);
  bool topicStatis(TildePathConfig & pinfo, double & s_stamp, double & cur_ros);
  void cbStatisEnter(const char *func);
  void cbStatisExit(const char *func);
  void cmdShowStatis();

  // others (debug)
  void log(std::string fs);
  void printLog();
  void enLog(bool ope);
  void dispLogCtrl(bool ope);
  void ajustPseudoRosTime();
  void pseudoRosTimeInit();
};

} // namespace tilde_timing_monitor
#endif  // TILDE_AGGREGATOR__TILDE_AGGREGATOR_CORE_HPP_
