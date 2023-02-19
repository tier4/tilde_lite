# tilde timing monitor (cpp)

## Detect timing violation specification

Refer to [the design page](../../docs/design_timing_violation_detection.md) for further details

## limitation

A measurable path is only if the topic of the end point node holds the stamp of the start point node.In that case, the response time is from the stamp of the topic to the time when the measurement node subscribes it.
If you use the TILDE message tracking tag, the response time will be the time until the topic is published, so it will be more accurate.

---

## environment

- ros2: humble
- AWF
  It is necessary to prepare an AWF environment that reflects <https://github.com/orgs/autowarefoundation/discussions/3176>.

  - fork of autoware foundation to TILDE

    - repo: <https://github.com/xygyo77/tilde-autoware.git>
    - branch: feat/add-ndt-publish-mtt-at-interpolation

  - <https://github.com/nabetetsu/tier4_autoware_msgs>
    - branch: feat/add_timing_violation_monitor_msgs

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

````yaml
/**:
  ros__parameters:
    diag_period_sec: 5.0 # frequency of report
    target_paths:
      ekf-to-ndt: # path name
        topic: /localization/pose_estimator/for_tilde_interpolator_mtt # topic name
        message_type: tilde_msg/msg/MessageTrackingTag # message type
        severity: warn # severity
        period: 100.0 # execution frequency of path
        deadline: 200.0 # deadline of response time
        violation_count_thresh: 2 # threshold to judge warn or not.

      pointcloudPreprocessor-to-ndt: # path name
        topic: /localization/pose_estimator/pose_with_covariance # topic name
        message_type: geometry_msgs/msg/PoseWithCovarianceStamped # message type
        severity: error # severity
        period: 100.0 # execution frequency of path
        deadline: 150.0 # deadline of response time
        violation_count_thresh: 1 # threshold to judge error or not.```
````

| name                   | content                              |
| ---------------------- | ------------------------------------ |
| diag_period_sec        | frequency of report (sec)            |
| target_paths           | target path information              |
| path_name(any word)    | the name given to the path           |
| topic name             | end point node published topic name  |
| message_type           | message type                         |
| severity               | diagnostic severity                  |
| periodic               | periodic timer value (msec)          |
| deadline               | deadline detect timer value (msec)   |
| violation_count_thresh | threshold to judge warn/error or not |

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

note: Deadline misses do not normally occur. In order to generate it artificially, for example, after starting the rosbag, it is necessary to apply a load with the stress command.

- tilde timing monitor

```bash
source ~/colcon_ws/tilde_lite/install/local_setup.bash
cp ~/colcon_ws/tilde_lite/src/tilde_timing_monitor/config/tilde_path_info.yaml .
ros2 launch tilde_timing_monitor tilde_timing_monitor_node.launch.xml config_file:=tilde_path_info.yaml
```

| param | value | content       | default |
| ----- | ----- | ------------- | ------- |
| debug | bool  | debug control | false   |

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
  - path_name: EKF to NDT pose
    topic: /localization/pose_estimator/for_tilde_interpolator_mtt
    valid_topic_count: 188
    discard_topic_count: 54
    completed_count: 187
    deadline_miss_count: 59
    false_deadline_miss_count: 0
    presumed_deadline_miss_count: 0
    response_count: 242
    response_time_min: 0.0808413028717041
    response_time_ave: 0.19864653752855033
    response_time_max: 0.7424073219299316
    too_long_response_count: 54
    too_long_response_time_min: 0.2438514232635498
    too_long_response_time_ave: 0.5194471544689603
    too_long_response_time_max: 0.7424073219299316
    hz_min: 0.08118677139282227
    hz_ave: 0.10450747794630122
    hz_max: 0.20474576950073242
    sub_min: 0.010127067565917969
    sub_ave: 0.10454824851261629
    sub_max: 0.285172700881958
    path_i: 0
    cur_j: 575
    completed_j: 573
    r_i_j_1_stamp:
      sec: 1585897285
      nanosec: 88705498
    r_i_j_1_float: 1585897285.0887055
    r_i_j_float: 1585897285.1887054
  - path_name: PointcloudPreprocessor to NDT
    topic: /localization/pose_estimator/pose_with_covariance
    valid_topic_count: 184
    discard_topic_count: 57
    completed_count: 183
    deadline_miss_count: 65
    false_deadline_miss_count: 0
    presumed_deadline_miss_count: 0
    response_count: 241
    response_time_min: 0.10250616073608398
    response_time_ave: 0.2228888040756289
    response_time_max: 0.8208935260772705
    too_long_response_count: 57
    too_long_response_time_min: 0.16010665893554688
    too_long_response_time_ave: 0.5512506585372122
    too_long_response_time_max: 0.8208935260772705
    hz_min: 0.09191489219665527
    hz_ave: 0.10458766619364421
    hz_max: 0.20739984512329102
    sub_min: 0.010118722915649414
    sub_ave: 0.10455933610598246
    sub_max: 0.3766031265258789
    path_i: 1
    cur_j: 608
    completed_j: 606
    r_i_j_1_stamp:
      sec: 1585897285
      nanosec: 79973000
    r_i_j_1_float: 1585897285.079973
    r_i_j_float: 1585897285.179973
cb_latency:
  - cb_name: onCommand
    cb_count: 2
    cb_min: 0.000100088
    cb_max: 0.068340274
    cb_ave: 0.034220181
  - cb_name: onDeadlineTimer
    cb_count: 124
    cb_min: 2.0789e-05
    cb_max: 0.000735932
    cb_ave: 0.00018947494354838715
  - cb_name: onPeriodicTimer
    cb_count: 474
    cb_min: 4.999e-06
    cb_max: 0.00147837
    cb_ave: 8.177309071729963e-05
  - cb_name: onIntervalTimer
    cb_count: 363
    cb_min: 7.644e-06
    cb_max: 0.000509256
    cb_ave: 5.889607438016527e-05
  - cb_name: onGenTopic
    cb_count: 483
    cb_min: 1.5679e-05
    cb_max: 0.001050526
    cb_ave: 0.00018876430227743278
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
--- statistics & infos topic ---
debug=true log=true disp=false

----- statistics (v0.12) -----
mode=test
path_name=EKF to NDT pose path_i=0 p_i=100(ms) d_i=200(ms)
topic=/localization/pose_estimator/for_tilde_interpolator_mtt [tier4_system_msgs/msg/MessageTrackingTag]
deadline detect=true
topic valid=188 discard=54
path OK=187 NG=59
path completed=187
deadline miss=59 false_miss=0 presumed miss=0
response time(242) min=0.080841 ave=0.198647 max=0.742407 (sec)
too long response time(54) min=0.243851 ave=0.519447 max=0.742407 (sec)
cur_j=575 completed_j=573
r_i_j_1=1585897285.088706 r_i_j=1585897285.188705
topic(241) HZ min=0.081187 ave=0.104507 max=0.204746 (sec) d_i over=1 per limit=1
topic(241) Sub interval min=0.010127 ave=0.104548 max=0.285173 (sec) d_i over=2 per limit=2
communication delay(242) min=0.000000 ave=0.000000 max=0.000000 (sec)
-- deadline timer ---
---------------------
path_name=PointcloudPreprocessor to NDT path_i=1 p_i=100(ms) d_i=150(ms)
topic=/localization/pose_estimator/pose_with_covariance [geometry_msgs/msg/PoseWithCovarianceStamped]
deadline detect=true
topic valid=184 discard=57
path OK=183 NG=65
path completed=183
deadline miss=65 false_miss=0 presumed miss=0
response time(241) min=0.102506 ave=0.222889 max=0.820894 (sec)
too long response time(57) min=0.160107 ave=0.551251 max=0.820894 (sec)
cur_j=608 completed_j=606
r_i_j_1=1585897285.079973 r_i_j=1585897285.179973
topic(240) HZ min=0.091915 ave=0.104588 max=0.207400 (sec) d_i over=2 per limit=2
topic(240) Sub interval min=0.010119 ave=0.104559 max=0.376603 (sec) d_i over=15 per limit=16
communication delay(241) min=0.000000 ave=0.000000 max=0.000000 (sec)
-- deadline timer ---
---------------------
--- callbacks ---
[onCommand] (3) min=0.000100 ave=0.022863 max=0.068340 (sec)
[onDeadlineTimer] (124) min=0.000021 ave=0.000189 max=0.000736 (sec)
[onPeriodicTimer] (474) min=0.000005 ave=0.000082 max=0.001478 (sec)
[onIntervalTimer] (363) min=0.000008 ave=0.000059 max=0.000509 (sec)
[onGenTopic] (483) min=0.000016 ave=0.000189 max=0.001051 (sec)
(END)-----------------

```
