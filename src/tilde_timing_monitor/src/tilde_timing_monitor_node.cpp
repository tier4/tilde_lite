#include "tilde_timing_monitor/tilde_timing_monitor_core.hpp"

#include <signal.h>

#include <rclcpp/rclcpp.hpp>

static void signal_handler(int signum) {
    printf("signal_handler: caught signal %d\n", signum);
    if (signum == SIGINT) {
        printf("--- SIGINT ---\n");
        exit(1);
    }
}

int main(int argc, char ** argv)
{
  if (signal(SIGINT, signal_handler) == SIG_ERR) {
    printf("## Error caught signal\n");
    exit(-1);
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tilde_timing_monitor::TildeTimingMonitor>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
