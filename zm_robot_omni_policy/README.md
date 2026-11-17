# zm_robot_omni_policy

zm_robot(麥克納姆輪)的全向導航策略:**凍結的 X-Mobility 基底 + SAC 殘差**,輸出 `/cmd_vel` 的 `linear.x`、`linear.y`、`angular.z`。

```
最終指令 (vx, vy, wz) = clip( X-Mobility 基底 [vx, 0, wz] + SAC 殘差 )
```

訓練在 `~/nvidia/zm_robot_omni_training`(Isaac Lab + Stable-Baselines3 SAC)。設計與實驗記錄見 [DESIGN.md](DESIGN.md),部署現況與驗證結果見其第 11 節。

**目前狀態:只在模擬資料的重播上驗證過(開迴路),還沒有在 Isaac Sim 閉迴路或實機上跑過。**

## 內容

| 路徑 | 說明 |
| --- | --- |
| `zm_robot_omni_policy/omni_policy_node.py` | 推論節點 |
| `zm_robot_omni_policy/preprocess.py` | 觀測前處理(訓練端觀測項的 numpy 版,不依賴 ROS) |
| `zm_robot_omni_policy/trt_runner.py` | TensorRT 引擎包裝(pycuda) |
| `zm_robot_omni_policy/replay_validation_node.py` | 重播驗證節點 |
| `launch/zm_robot_omni_policy.launch.py` | 執行策略 |
| `launch/replay_validation.launch.py` | 執行重播驗證 |
| `scripts/build_engines.sh` | ONNX → TensorRT 引擎 |
| `scripts/check_engines_offline.py` | 不經 ROS 的引擎比對 |
| `models/` | 模型與權重(見下) |

> **2026-11-17:** `models/` 已整個移出這個 package,現在在 `~/nvidia/zm_robot_omni_training/deploy_models/`(內容不變,36 個檔案已用 md5 核對)。啟動節點時要指定位置,例如 `model_dir:=~/nvidia/zm_robot_omni_training/deploy_models`,或設定環境變數 `ZM_ROBOT_OMNI_MODEL_DIR`;`scripts/build_engines.sh`、`scripts/check_engines_offline.py` 與重播驗證也要指向那裡。

### `models/`

| 檔案 | 說明 |
| --- | --- |
| `xmobility.onnx` / `.engine` | 凍結的 X-Mobility,單步(含 RSSM 狀態進出) |
| `sac_residual_actor_{hybrid,penalty,progress}.onnx` / `.engine` | SAC 殘差 actor(三種朝向獎勵各一) |
| `export_meta.json` | 匯出時的輸入形狀、殘差範圍、ONNX 對 PyTorch 的差距 |
| `checkpoints/sac_resid_*_seed101_150000_steps.zip` | SB3 SAC 原始檢查點(訓練 15 萬步) |
| `x_mobility/` | X-Mobility 的模型程式、gin 設定與 PyTorch 權重(重新匯出時才需要) |
| `validation/ref_hybrid.npz` | 訓練環境錄下的參考回合(156 步,原始感測器 + PyTorch 輸出) |
| `validation/replay_report*.json` | 最近一次驗證報告 |

`.engine` 綁定建立它的 GPU 與 TensorRT 版本,換機器要重建。模型共約 1.2 GB,**不會**被 colcon 複製到 install;節點依序用 `model_dir` 參數、`$ZM_ROBOT_OMNI_MODEL_DIR`、原始碼目錄的 `models/` 找模型。

## 需求

ROS 2 Jazzy、TensorRT 的 Python 綁定、`pycuda`、`cv_bridge`、`tf2_ros`、`tf2_geometry_msgs`。

## 建置

```bash
cd ~/catkin_ws && colcon build --packages-select zm_robot_omni_policy
```

```bash
~/catkin_ws/src/zm_robot/zm_robot_omni_policy/scripts/build_engines.sh
```

## 執行

```bash
ros2 launch zm_robot_omni_policy zm_robot_omni_policy.launch.py
```

在 RViz 用 2D Goal Pose 發 `/goal_pose` 後開始輸出指令;每個新目標會重置策略的內部狀態。常用參數:

| 參數 | 預設 | 說明 |
| --- | --- | --- |
| `actor` | `hybrid` | `hybrid` / `penalty` / `progress` |
| `sync_mode` | `timer` | `timer`:以 `control_rate` 執行(實機)。`stamp`:每個感測器時間戳執行一次(模擬) |
| `control_rate` | `10.0` | Hz,訓練的控制週期是 0.1 s |
| `sensor_qos` | `best_effort` | 感測器訂閱的 QoS |
| `stop_at_goal` | `true` | 進入到位容許範圍(0.5 m、0.5 rad)時送出一次零速度讓車停下,之後不再發布 `/cmd_vel`,直到收到新目標 |
| `max_linear_x` / `max_linear_y` / `max_angular_z` | 0.6 / 0.6 / 1.57 | 指令上限 |
| `max_linear_accel` / `max_angular_accel` | 2.0 / 3.14 | `/cmd_vel` 的加速度上限(m/s²、rad/s²),和訓練環境相同;設 0 關閉 |
| `cmd_publish_rate` | 50.0 | 加速度限制啟用時 `/cmd_vel` 的發布頻率(Hz);策略本身仍是 10 Hz |
| `enable_map_tf_broadcaster` | `true` | 用 `/odom` 廣播 `map → base_link`(沒有 SLAM 時用;接上 AMCL 或 Cartographer 時設 `false`) |

### 介面

| 方向 | Topic | 型別 |
| --- | --- | --- |
| 訂閱 | `/kinect_v2/color/image_raw` | `sensor_msgs/Image` |
| 訂閱 | `/kinect_v2/depth/image_raw` | `sensor_msgs/Image`(32FC1 公尺或 16UC1 公釐) |
| 訂閱 | `/sick_lidar0/scan`、`/sick_lidar1/scan` | `sensor_msgs/LaserScan` |
| 訂閱 | `/odom` | `nav_msgs/Odometry`(用 twist) |
| 訂閱 | `/goal_pose` | `geometry_msgs/PoseStamped` |
| 訂閱 | `/emergency_stop` | `std_msgs/Bool`(true 時輸出零速度;給外部安全裝置用,這個 package 本身沒有節點發布它) |
| 發布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 發布 | `~/cmd_vel_stamped`、`~/debug` | 除錯用(欄位見 `interface.py`) |

另外需要 TF:目標所在座標系 → `base_link`,以及兩顆 LiDAR 的座標系 → `base_link`。

**加速度限制:**`timer` 模式下,`/cmd_vel` 不會直接跳到策略的指令,而是以 50 Hz 依加速度上限逐步逼近。訓練環境在策略指令和機器人之間就有同樣的限制(2.0 m/s²、3.14 rad/s²),少了它,速度指令會以階梯方式送出(例如 0 直接到 0.6 m/s),在 Isaac Sim 裡會造成明顯的衝擊(2026-11-10 實測:起步時 IMU 的高頻加速度達 9.8 m/s²,其中垂直方向 9.2、水平 5.3)。到達目標時也是先依這個上限減速到零,再停止發布。緊急停止和感測器逾時則是立刻歸零,不經過減速。`~/cmd_vel_stamped` 仍是策略的原始輸出。

安全行為:`timer` 模式下任一感測器超過 `stale_timeout`(0.5 s)沒有新資料就輸出零速度;節點結束時送出一次零速度;`/emergency_stop` 為 true 時持續輸出零速度。

**沒有 IMU 安全監控。**曾經做過 `imu_safety_monitor_node`(用 `/imu` 偵測傾斜與衝擊並發布 `/emergency_stop`),但門檻沒有校正,在 Isaac Sim 裡太容易誤觸發急停,已於 2026-11-11 移除。目前的安全機制只有感測器逾時停車,以及上面這個可由外部發布的 `/emergency_stop`。

## 驗證

**重播驗證(逐步比對):**把訓練環境錄下的回合當成 ROS2 topic 發布,比對節點的指令與 PyTorch 策略的輸出。

```bash
ros2 launch zm_robot_omni_policy replay_validation.launch.py
```

**`timer` 模式冒煙測試**(10 Hz 即時發布,檢查指令頻率與感測器中斷時是否停車):

```bash
ros2 launch zm_robot_omni_policy replay_validation.launch.py mode:=realtime
```

**不經 ROS 的引擎比對:**

```bash
python3 scripts/check_engines_offline.py --actor hybrid --zero_route_first_step
```

2026-11-10 在本機(RTX 5070 Ti、TensorRT 11.4、FP32)的結果:

| 項目 | 結果 |
| --- | --- |
| 最終指令對 PyTorch,147 個參考一致的步 | 最大差 0.0031、平均 0.0004(m/s、rad/s);門檻 0.02 |
| 全部 156 步 | 最大差 0.114,都落在被排除的 9 步 |
| 前處理後的觀測對模擬器觀測 | 差 ≤ 1e-6;深度圖個別像素差 1/255 |
| 三個 actor 引擎對 PyTorch 殘差 | 最大差 ≤ 0.0004 |
| 推論時間(前處理 + 兩個引擎) | 平均 10.1 ms、p95 12.7 ms |
| `timer` 模式 | 9.8 Hz;感測器停止後輸出零速度 |

被排除的 9 步是每個回合的前 3 步:模擬器在回合第一步的觀測本身不一致(LiDAR 還在重置前的位置;整次執行的第一步 X-Mobility 拿到全零路線),接下來 2 步受前一步指令與 RSSM 狀態影響。細節見 DESIGN.md 第 11 節。

**這個驗證不包含:**閉迴路行駛、實機感測器(Kinect 視角與解析度、SICK 270° 視角與車身遮擋)、輪子動力學。

## 重新匯出模型

在訓練用的容器內(`~/nvidia/zm_robot_omni_training`):

```bash
python3 scripts/export_deploy.py --out deploy_export --checkpoint hybrid=logs/sb3/zm_robot_omni_sac_state_lidar_xmob/resid_hybrid_seed101/model_150000_steps.zip
```

```bash
python3 scripts/record_deploy_ref.py --checkpoint logs/sb3/zm_robot_omni_sac_state_lidar_xmob/resid_hybrid_seed101/model_150000_steps.zip --reward hybrid --out deploy_export/ref_hybrid.npz --episodes 3 --headless --enable_cameras
```

把 `deploy_export/` 裡的 `.onnx`、`export_meta.json` 放到 `models/`,`ref_hybrid.npz` 放到 `models/validation/`,再重建引擎。
