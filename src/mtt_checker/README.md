# tilde MTT checker

tilde_timing_monitor の観測結果の妥当性をチェックするツール。
NDT-interporate 時の MTT トピックを記録したファイルを用意し、MTT に収集した EKF-stamp 情報から静的に解析する。
入力には、MTT を記録した rosbag ファイル、あるいは`ros2 topic echo target-mtt-topic`で収集した mtt トピックの yaml ファイルを使用する。

## environment & install

tilde_timing_monitor に同じ。

## sample

- STRESS-rosbag  
起動15秒後にstressコマンドで負荷を掛けてデッドラインミスを発生させてrosbagに記録    

## operation

source コマンドで、ros2、AWF および tilde_lite 環境を読み込む。

```
ros2 run mtt_checker mtt_checker -h
usage: ros2 run mtt_checker mtt_checker [-h] [-m mode] [-p time] [-d time] [-t name] input file

Check the deadline by Message tracking tag (mtt).

positional arguments:
  input                 Input rosbag or MTT yaml file

options:
  -h, --help            show this help message and exit
  -m mode, --mode mode  simple: check stamp only, normal: default
  -p time, --periodic time
                        periodic time default:100.0 (ms)
  -d time, --deadline time
                        deadline detect time default:200.0 (ms)
  -t name, --topic name
                        topic name: Specify the target topic if there are multiple MTTs in the rosbag. default: for_tilde_interpolator_mtt
```

出力(結果は最終行辺り)  
**シンプルモード**  
MTT の stamp とその間隔のみで簡易的にデッドラインミスを検出する

```
ros2 run mtt_checker mtt_checker ~/tilde_lite/src/mtt_checker/sample/STRESS-rosbag/ -m simple -p 100 -d 200 -t for_tilde_interpolator_mtt 
### DEBUG=False
--- STAR### DEBUG=False
[WARN] [1674613753.225416499] [rosbag2_storage]: No storage plugin found with id 'sqlite3'.
[INFO] [1674613753.226777873] [rosbag2_storage]: Opened database '/home/akm/tilde_lite/src/mtt_checker/sample/STRESS-rosbag/STRESS-rosbag_0.db3' for READ_ONLY.
--- START (simple: /home/akm/tilde_lite/src/mtt_checker/sample/STRESS-rosbag/) ---
    periodic_time=100.0(ms) deadline_time=200.0(ms)

MTT: /localization/pose_estimator/for_tilde_interpolator_mtt
SRC: /localization/pose_twist_fusion_filter/biased_pose_with_covariance
release_time: SRC topic stamp
interval1(s): SRC(n+1)-SRC(n) topic stamp interval
pub_time: MTT topic publish time
interval2(s): MTT(n+1)-MTT(n)) topic stamp interval
proc_time(s): pub_time - release_time(EKF+NDT process time)
interval(s): SRC(n+1)-SRC(n) topic stamp interval
proc_time(s): pub_time - release_time(EKF+NDT process time)
| No |  release_time   |interval1|     pub_time    |interval2|proc_time|deadline decision             |
|   1|1585897258.947767| 0.000000|1585897259.059858| 0.000000| 0.112091|OK.(Under 0.2)|
|   2|1585897259.049708| 0.101941|1585897259.161084| 0.101226| 0.111376|OK.(Under 0.2)|
|   3|1585897259.161084| 0.111376|1585897259.292917| 0.131833| 0.131833|OK.(Under 0.2)|
(snip)
| 246|1585897284.569229| 0.101162|1585897285.189880| 0.000000| 0.620651|Deadline miss.(Over 0.2 (  3))|
| 247|1585897284.680522| 0.111293|1585897285.189880| 0.000000| 0.509358|Deadline miss.(Over 0.2 (  2))|
| 248|1585897284.782763| 0.102240|1585897285.189880| 0.000000| 0.407118|Deadline miss.(Over 0.2 (  2))|
--- OK=163 Deadline miss=224 mtt topic=247 ---------------------------------
(END:v0.12)---------------------------------
```
**ノーマルモード**  
シンプルモードに加えて、periodic タイマ毎にデッドライン検出タイマを起動させた時のシミュレーションを行う。通信遅延は考慮されない。
```
ros2 run mtt_checker mtt_checker ~/tilde_lite/src/mtt_checker/sample/STRESS-rosbag/ -m normal -p 100 -d 200 -t for_tilde_interpolator_mtt 
### DEBUG=False
[WARN] [1674613470.464989912] [rosbag2_storage]: No storage plugin found with id 'sqlite3'.
[INFO] [1674613470.466455932] [rosbag2_storage]: Opened database '/home/akm/tilde_lite/src/mtt_checker/sample/STRESS-rosbag/STRESS-rosbag_0.db3' for READ_ONLY.
--- START (normal: /home/akm/tilde_lite/src/mtt_checker/sample/STRESS-rosbag/) ---
    periodic_time=100.0(ms) deadline_time=200.0(ms)

MTT: /localization/pose_estimator/for_tilde_interpolator_mtt
ORG: /localization/pose_twist_fusion_filter/biased_pose_with_covariance
release_time: SRC topic stamp
dur-1(s): SRC(n+1)-SRC(n) topic stamp interval
pub_time: MTT topic publish time
dur-2(s): MTT(n+1)-MTT(n)) topic stamp interval
proc(s): pub_time - release_time (response_time)
| No |  release_time   |  dur-1 |     pub_time    |  dur-2 |  proc  |   |   OK     |   NG     | remarks               |
|   1|1585897258.947767|0.000000|1585897259.059858|0.000000|0.112091|  0|ana.ok=  0|ana.ng=  0|||
|-p_i|1585897259.047767|0.100000|1585897259.247767|        |        |  0|ana.ok=  0|ana.ng=  0|periodic+deadline START|
|   2|1585897259.049708|0.101941|1585897259.161084|0.101226|0.111376|  1|ana.ok=  0|ana.ng=  0|||
|    |                 |        |1585897259.047767|        |        |  1|ana.ok=  1|ana.ng=  0|OK|
|-p_i|1585897259.149708|0.100000|1585897259.349708|        |        |  0|ana.ok=  1|ana.ng=  0|periodic+deadline START|
|   3|1585897259.161084|0.111376|1585897259.292917|0.131833|0.131833|  1|ana.ok=  1|ana.ng=  0|||
|    |                 |        |1585897259.149708|        |        |  1|ana.ok=  2|ana.ng=  0|OK|
(snip)
|-p_i|1585897284.461586|8.699992|1585897284.661587|        |        |  1|ana.ok=152|ana.ng= 87|periodic+deadline TIMEOUT|
|-p_i|1585897284.561586|8.799992|1585897284.761586|        |        |  1|ana.ok=152|ana.ng= 88|periodic+deadline TIMEOUT|
|-p_i|1585897284.661586|8.899992|1585897284.861586|        |        |  1|ana.ok=152|ana.ng= 89|periodic+deadline TIMEOUT|
|-p_i|1585897284.761586|8.999991|1585897284.961586|        |        |  1|ana.ok=152|ana.ng= 90|periodic+deadline TIMEOUT|
--- p_i=100.00(ms) d_i=200.00(ms) ---
--- p_i count= 246 p_i in time=258.350 ---
--- ( 248) OK:152 NG: 90 completed_j=242 ---
(END:v0.12)---------------------------------```
