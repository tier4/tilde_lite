# tilde timing monitor (cpp)

## Detect timing violation specification

Refer to [the design page](./design_timing_violation_detection.md) for further details

## limitation

A measurable path is only if the topic of the end point node holds the stamp of the start point node.In that case, the response time is from the stamp of the topic to the time when the measurement node subscribes it.
If you use the TILDE message tracking tag, the response time will be the time until the topic is published, so it will be more accurate.

---

## environment

- ros2: humble
- fork of autoware foundation to TILDE

  - repo: <https://github.com/xygyo77/tilde-autoware.git>
  - branch: humble

  ```bash
  vcs import src << nrm-build.hashed.repos
  ```

note: ndt_scan_matcher contains a unique change that outputs the latest EKF pose topic used during interpolation.

## install

- repo: <https://github.com/tier4/tilde_lite.git>

```bash
cd ~/colcon_ws
git clone https://github.com/tier4/tilde_lite.git
cd tilde_lite
colcon build --symlink-install
```

- source ROS2/autoware environments

- cyclone dds parameters
  Extend receive buffer size.

cyclonedds_config.xml

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
        </Internal>
    </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file:///absolute/path/to/cyclonedds_config.xml
sudo sysctl -w net.core.rmem_max=2147483647
```

See <https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/#tuning-dds>

- prepare path list yaml file (see. config/tilde_path_info.yaml)

```yaml
/**:
  ros__parameters:
    required_paths:
      test:
        /localization/pose_estimator/for_tilde_interpolator_mtt:
          {
            mtype: "tilde_msg/msg/MessageTrackingTag",
            path_name: "EKF=>NDT",
            path_i: 0,
            p_i: 100.0,
            d_i: 150.0,
            level: warn,
          }
        /localization/pose_estimator/pose_with_covariance:
          {
            mtype: "geometry_msgs/msg/PoseWithCovarianceStamped",
            path_name: "PCL=>NDT",
            path_i: 1,
            p_i: 100.0,
            d_i: 120.0,
            level: warn,
          }
```

| name                | content                             |
| ------------------- | ----------------------------------- |
| required_paths      | measurement path information        |
| mode (ex. test)     | measurement path set id             |
| topic name          | end point node published topic name |
| mtype               | message type                        |
| path_name(any word) | the name given to the path          |
| path_i              | path number                         |
| p_i                 | periodic timer value (msec)         |
| d_i                 | deadline detect timer value (msec)  |
| level               | diagnostic severity                 |

- invoke logging simulator

```bash
ros2 launch autoware_launch logging_simulator.launch.xml   map_path:=/home/akm/data/sample-map-rosbag   vehicle_model:=sample_vehicle   sensor_model:=sample_sensor_kit rviz:=True
...
(wait for the nodes to come up...)
...
```

- other terminal

```bash
ros2 bag play /home/akm/data/sample-rosbag -r 0.2
```

- tilde timing monitor

```bash
source ~/colcon_ws/tilde_lite/install/local_setup.bash
cp ~/colcon_ws/tilde_lite/src/tilde_timing_monitor/config/tilde_path_info.yaml .
ros2 launch tilde_timing_monitor tilde_timing_monitor_node.launch.xml config_file:=tilde_path_info.yaml mode:=test
```

| param      | value          | content                       | default |
| ---------- | -------------- | ----------------------------- | ------- |
| mode       | by config file | Measurement path type         | test    |
| debug      | bool           | debug  control                | false   |

## output

- **deadline miss detected topic**

/tilde_timing_monitor/output/tilde_timing_monitor/deadline_miss

```yaml
header:
  stamp:
    sec: 1585897263
    nanosec: 609890262
  frame_id: ""
path_name: EKF=>NDT
topic: /localization/pose_estimator/for_tilde_interpolator_mtt
deadline_timer: 0.2
periodic_timer: 0.1
path_i: 0
self_j: 55
cur_j: 58
completed_j: 55
last_r_i_j_1_stamp:
  sec: 1585897263
  nanosec: 304774761
last_r_i_j_1_float: 1585897263.3047748
deadline_timer_start_stamp:
  sec: 1585897263
  nanosec: 404774665
deadline_timer_start: 1585897263.4047747
mode: test
```

- **information and statistics topic**

```bash
ros2 topic pub  /tilde_timing_monitor_command tilde_timing_monitor_interfaces/msg/TildeTimingMonitorCommand '{command: req info}' --once
```

```yaml
header:
  stamp:
    sec: 1585897285
    nanosec: 189880315
  frame_id: ""
mode: test
path_info:
  - path_name: EKF=>NDT
    topic: /localization/pose_estimator/for_tilde_interpolator_mtt
    completed_count: 211
    deadline_miss_count: 22
    false_deadline_miss_count: 0
    presumed_deadline_miss_count: 0
    response_count: 211
    response_time_min: 0.0713193416595459
    response_time_ave: 0.10486668998031255
    response_time_max: 0.13193845748901367
    too_long_response_count: 0
    too_long_response_time_min: 0.0
    too_long_response_time_ave: 0.0
    too_long_response_time_max: 0.0
    valid_topic_count: 230
    discard_topic_count: 0
    hz_min: 0.07070088386535645
    hz_ave: 0.11405150994010593
    hz_max: 0.386655330657959
    path_i: 0
    cur_j: 271
    completed_j: 269
    r_i_j_1_stamp:
      sec: 1585897285
      nanosec: 88705498
    r_i_j_1_float: 1585897285.0887055
    r_i_j_float: 1585897285.1887054
  - path_name: PCL=>NDT
    topic: /localization/pose_estimator/pose_with_covariance
    completed_count: 228
    deadline_miss_count: 22
    false_deadline_miss_count: 0
    presumed_deadline_miss_count: 0
    response_count: 228
    response_time_min: 0.10034441947937012
    response_time_ave: 0.12089436096057557
    response_time_max: 0.14562726020812988
    too_long_response_count: 1
    too_long_response_time_min: 0.15234684944152832
    too_long_response_time_ave: 0.0
    too_long_response_time_max: 0.15234684944152832
    valid_topic_count: 228
    discard_topic_count: 0
    hz_min: 0.09191489219665527
    hz_ave: 0.11372711783961247
    hz_max: 0.4174771308898926
    path_i: 1
    cur_j: 253
    completed_j: 251
    r_i_j_1_stamp:
      sec: 0
      nanosec: 0
    r_i_j_1_float: 1585897285.079973
    r_i_j_float: 1585897285.179973
cb_latency:
  - cb_name: onCommand
    cb_count: 1
    cb_min: 0.082034712
    cb_max: 0.082034712
    cb_ave: 0.082034712
  - cb_name: onDeadlineTimer
    cb_count: 44
    cb_min: 2.3735e-05
    cb_max: 8.1393e-05
    cb_ave: 3.929977272727273e-05
  - cb_name: onGenTopic
    cb_count: 230
    cb_min: 8.4539e-05
    cb_max: 0.006083014
    cb_ave: 0.00017973195652173907
  - cb_name: onMttTopic
    cb_count: 231
    cb_min: 6.543e-06
    cb_max: 0.000376577
    cb_ave: 0.00010830023809523808
  - cb_name: onPeriodicTimer
    cb_count: 5127
    cb_min: 8.0e-08
    cb_max: 0.001770122
    cb_ave: 9.050668031987475e-06
```

## command

Publish tilde_timing_monitor commands as below.

| command   | contents                             |
| --------- | ------------------------------------ |
| req info  | publish infos and statistics topic   |
| show info | show infos and statistics on console |
| show hist | show log                             |
| dbgon     | debug mode enable                    |
| dbgoff    | debug mode disable                   |
| clrinfo   | clear statistics & information       |
| detoff    | detect deadline disable -> enable    |

- **example show info**

```bash
ros2 topic pub  /tilde_timing_monitor_command tilde_timing_monitor_interfaces/msg/TildeTimingMonitorCommand '{command: show info}' --once
```

```bash
----- statistics (v0.01) -----
mode=test
path_name=EKF=>NDT path_i=0 p_i=100(ms) d_i=200(ms)
topic=/localization/pose_estimator/for_tilde_interpolator_mtt [tilde_msg/msg/MessageTrackingTag]
path completed=211 presumed completed=0
deadline miss=22 false miss=0 presumed miss=0
response time(211) min=0.0713193416595459 ave=0.10486668998031255 max=0.13193845748901367 (sec)
too long response time(0) min=0 ave=0 max=0 (sec)
cur_j=271 completed_j=269
r_i_j_1=1585897285.0887055 r_i_j=1585897285.1887054
topic(230) HZ min=0.07070088386535645 ave=0.11405150994010593 max=0.386655330657959 (sec)
-- deadline timer ---
---------------------
path_name=PCL=>NDT path_i=1 p_i=100(ms) d_i=150(ms)
topic=/localization/pose_estimator/pose_with_covariance [geometry_msgs/msg/PoseWithCovarianceStamped]
path completed=228 presumed completed=0
deadline miss=22 false miss=0 presumed miss=0
response time(228) min=0.10034441947937012 ave=0.12089436096057557 max=0.14562726020812988 (sec)
too long response time(1) min=0.15234684944152832 ave=0.15234684944152832 max=0.15234684944152832 (sec)
cur_j=253 completed_j=251
r_i_j_1=1585897285.079973 r_i_j=1585897285.179973
topic(228) HZ min=0.09191489219665527 ave=0.11372711783961247 max=0.4174771308898926 (sec)
-- deadline timer ---
---------------------
--- callbacks ---
[onCommand] (2) min=0.000127309 ave=0.0410810105 max=0.082034712 (sec)
[onDeadlineTimer] (44) min=2.3735e-05 ave=3.929977272727273e-05 max=8.1393e-05 (sec)
[onGenTopic] (230) min=8.4539e-05 ave=0.00017973195652173907 max=0.006083014 (sec)
[onMttTopic] (231) min=6.543e-06 ave=0.00010830023809523808 max=0.000376577 (sec)
[onPeriodicTimer] (5127) min=8e-08 ave=9.050668031987475e-06 max=0.001770122 (sec)
(END)-----------------
```
