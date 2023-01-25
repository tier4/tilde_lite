# tilde MTT checker

tilde_timing_monitor の観測結果の妥当性をチェックするツール。
NDT-interporate時のMTTトピックを記録したファイルを用意し、MTT に収集した EKF-stamp 情報から静的に解析する。
入力には、MTTを記録したrosbagファイル、あるいは`ros2 topic echo target-mtt-topic`で収集したmttトピックのyamlファイルを使用する。

## environment & install

tilde_timing_monitor に同じ。

## sample

MTTトピックを収集したサンプルを sampleディレクトリ配下に配置した。

- tp-ekf-pose-PREV-AWF.yaml  
  旧AWF(10/28)ベース：f9ca032b226e5dd4a983f2e6171af32237b5911f で記録したMTT
- tp-ekf-pose-CUR-AWF.yaml  
  新AWF(12/22)ベース：d34355c8d5ce26405546c183405f6fa0f76e001e で記録したMTT

## operation

sourceコマンドで、ros2、AWF および tilde_lite 環境を読み込む。

```bash
ros2 run mtt_checker mtt_checker -h
usage: ros2 run mtt_checker mtt_checker [-h] [-m mode] [-p time] [-d time] [-t name] input file

Check the deadline by Message tracking tag (mtt).

positional arguments:
  input                 Input rosbag or MTT yaml file

options:
  -h, --help            show this help message and exit
  -m mode, --mode mode  simple: check stamp only, normal: default
  -p time, --periodic time
                        periodic time default:100.0 (ms) use normal mode only
  -d time, --deadline time
                        deadline detect time default:200.0 (ms)
  -t name, --topic name
                        topic name: Specify the target topic if there are
                        multiple MTTs in the rosbag. default:
                        for_tilde_interpolator_mtt

```

出力結果(最終行辺り)  
**シンプルモード**  
MTTのstampとその間隔のみで簡易的にデッドラインミスを検出する

```bash
ros2 run mtt_checker mtt_checker tp-ekf-pose-PREV-AWF.yaml -d 200 -m simple
### DEBUG=False
--- START (simple: tp-ekf-pose-PREV-AWF.yaml) ---
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
|   1|1585897258.643023| 0.000000|1585897258.724020| 0.000000| 0.080997|OK.(Under 0.2)|
|   2|1585897258.744415| 0.101392|1585897258.845908| 0.121888| 0.101493|OK.(Under 0.2)|
|   3|1585897258.845908| 0.101493|1585897258.947767| 0.101859| 0.101859|OK.(Under 0.2)|

(snip)

| 222|1585897284.884746| 0.315517|1585897284.987379| 0.316986| 0.102632|Deadline miss.(Over 0.2 (  1))|
| 223|1585897284.987379| 0.102632|1585897285.078572| 0.091194| 0.091194|OK.(Under 0.2)|
| 224|1585897285.088706| 0.101327|1585897285.189880| 0.111308| 0.101175|OK.(Under 0.2)|
--- OK=209 Deadline miss=17 mtt topic=224 ---------------------------------
(END:v0.1)---------------------------------
```

**ノーマルモード**  
シンプルモードに加えて、periodicタイマ毎にデッドライン検出タイマを起動させた時のシミュレーションを行う。通信遅延や、CPU負荷による時刻遅れは考慮されない。

```bash
ros2 run mtt_checker mtt_checker tp-ekf-pose-PREV-AWF.yaml -p 100 -d 200 -m normal
### DEBUG=False
--- START (normal: tp-ekf-pose-PREV-AWF.yaml) ---
    periodic_time=100.0(ms) deadline_time=200.0(ms)

MTT: /localization/pose_estimator/for_tilde_interpolator_mtt
ORG: /localization/pose_twist_fusion_filter/biased_pose_with_covariance
release_time: SRC topic stamp
interval1(s): SRC(n+1)-SRC(n) topic stamp interval
pub_time: MTT topic publish time
interval2(s): MTT(n+1)-MTT(n)) topic stamp interval
proc_time(s): pub_time - release_time(EKF+NDT process time)
interval(s): SRC(n+1)-SRC(n) topic stamp interval
proc_time(s): pub_time - release_time(EKF+NDT process time)
| No |  release_time   | dur-1  |     pub_time    |  due-2 |  proc  |   |   OK     |    P-OK    |   NG     |    P-NG    |  remarks               |
|   1|1585897258.643023|0.000000|1585897258.724020|0.000000|0.080997|  0|ana.ok=  0|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0||
|-p_i|1585897258.743023|0.100000|1585897258.943023|        |        |  0|ana.ok=  0|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0|periodic+deadline START|
|   2|1585897258.744415|0.101392|1585897258.845908|0.121888|0.101493|  1|ana.ok=  0|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0||
|    |                 |        |1585897258.743023|        |        |  1|ana.ok=  1|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0|OK|
|-p_i|1585897258.844414|0.100000|1585897259.044415|        |        |  0|ana.ok=  1|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0|periodic+deadline START|
|   3|1585897258.845908|0.101493|1585897258.947767|0.101859|0.101859|  1|ana.ok=  1|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0||
|    |                 |        |1585897258.844414|        |        |  1|ana.ok=  2|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0|OK|
|-p_i|1585897258.945908|0.100000|1585897259.145908|        |        |  0|ana.ok=  2|ana.p_ok=  0|ana.ng=  0|ana.p_ng=  0|periodic+deadline START|

(snip)

| 222|1585897284.884746|0.315517|1585897284.987379|0.316986|0.102632|  1|ana.ok=208|ana.p_ok=  0|ana.ng= 29|ana.p_ng=  0|skip: deadline miss occured|
|-p_i|1585897284.669229|0.100000|1585897284.869229|        |        |  1|ana.ok=208|ana.p_ok=  0|ana.ng= 30|ana.p_ng=  0|periodic+deadline TIMEOUT|
|-p_i|1585897284.769229|0.200000|1585897284.969229|        |        |  1|ana.ok=208|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|periodic+deadline TIMEOUT|
|-p_i|1585897284.869229|0.300000|1585897285.069229|        |        |  1|ana.ok=208|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|periodic+deadline START|
|-p_i|1585897284.969229|0.400000|1585897285.169229|        |        |  2|ana.ok=208|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|periodic+deadline START|
| 223|1585897284.987379|0.102632|1585897285.078572|0.091194|0.091194|  3|ana.ok=208|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0||
|    |                 |        |1585897284.568068|        |        |  3|ana.ok=209|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|OK|
|    |                 |        |1585897284.869229|        |        |  3|ana.ok=210|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|OK|
|    |                 |        |1585897284.969229|        |        |  3|ana.ok=211|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|OK|
|-p_i|1585897285.087379|0.100000|1585897285.287379|        |        |  0|ana.ok=211|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|periodic+deadline START|
| 224|1585897285.088706|0.101327|1585897285.189880|0.111308|0.101175|  1|ana.ok=211|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0||
| 224|1585897285.088706|0.101327|1585897285.189880|0.111308|0.101175|  1|ana.ok=211|ana.p_ok=  0|ana.ng= 31|ana.p_ng=  0|one before last line|
--- p_i=100.00(ms) d_i=200.00(ms) ---
--- p_i count= 224 p_i in time=264.457 ---
--- ( 224) OK:211 NG: 31 pseudo OK:  0 pseudo NG:  0 completed_j=242 ---
(END:v0.1)---------------------------------
```
