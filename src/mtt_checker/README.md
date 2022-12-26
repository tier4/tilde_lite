# tilde MTT checker
tilde_timing_monitor の観測結果の妥当性をチェックするツール。
NDT-interporate時のMTTトピックを記録したファイルを用意し、MTT に収集した EKF-stamp 情報から静的に解析する。

## environment & install
tilde_timing_monitor に同じ。

## sample
MTTトピックを収集したサンプルを sampleディレクトリ配下に配置した。
- tp-ekf-pose-PREV-AWF.yaml  
旧AWF(10/28)ベース：f9ca032b226e5dd4a983f2e6171af32237b5911f で記録したMTT
- tp-ekf-pose-CUR-AWF.yaml  
新AWF(12/22)ベース：d34355c8d5ce26405546c183405f6fa0f76e001e で記録したMTT

## operation
sourceコマンドで、ros2、AWFおよびtilde_timing_monitor環境を読み込む。
```
ros2 run mtt_checker mtt_checker tp-ekf-pose-CUR-AWF.yaml 100 200
```
結果は最終行辺り
```
--- p_i=100.00(ms) d_i=200.00(ms) ---
--- p_i count= 159 p_i in time=265.268 ---
--- ( 129) OK: 90 NG:155 pseudo OK:  0 pseudo NG:  0 completed_j=245 ---
```
NG がデッドラインミス検出数を示している。