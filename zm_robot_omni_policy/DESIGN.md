# zm_robot_omni_policy 架構設計文件

> **2026-11-10 更新(以這段為準):**目前部署的模型是「凍結 X-Mobility 基底 + SAC 殘差」(SB3 SAC,不是 RSL-RL PPO),已匯出成 ONNX/TensorRT,ROS2 package 與重播驗證已完成,見**第 11 節**與 `README.md`。下面第 0~9 節是較早的設計記錄,其中「不使用 X-Mobility」「RSL-RL 端到端」「LiDAR 占據圖 + CNN」等敘述已被第 10 節步驟 8 之後的實驗取代,保留作為歷史。

(以下為早期狀態)狀態:**已完全脫離 COMPASS/X-Mobility,且 RSL-RL PPO 訓練迴圈已跑通**。`zm_robot_omni_training` 零 `import mobility_es`/`import compass`,只依賴 Isaac Lab 本身(不同的 NVIDIA 產品,不算 COMPASS)+ RSL-RL(Isaac Lab 官方配套)。`train.py` 實際執行過 3 輪 PPO learning iteration,CNN(camera/lidar)+ MLP(policy 向量)multi-modal actor-critic 正確建構、forward/backward 都跑過,reward/metrics 正常記錄,沒有崩潰。

重大代價(已知、接受):沒有 X-Mobility 的凍結視覺骨幹可用(transfer learning 沒了),RGB/Depth 的 CNN encoder 要從零訓練,收斂速度預期比原計畫慢很多;COMPASS 的 `ResidualPPOTrainer`/distillation 架構整個不用,改成 RSL-RL 端到端訓練。

## 0. 程式碼實際位置

| 內容 | 路徑 | 性質 |
|---|---|---|
| 訓練側 Isaac Lab 專案 | **`/home/zmtech/nvidia/zm_robot_omni_training/`** | 獨立 repo,零依賴 COMPASS,只依賴 Isaac Lab + RSL-RL |
| COMPASS repo | 無任何整合點、無任何參照 | — |
| 部署側 ROS2 package | `zm_robot_omni_policy`(本目錄) | 尚待第 10 節步驟補上 `omni_policy_node`/`imu_safety_monitor_node` |

`zm_robot_omni_training` 實際檔案結構(已驗證可運作,含實際跑過訓練迴圈):
```
zm_robot_omni_training/
  docker.sh                              (COMPASS docker wrapper——Isaac Lab 用的還是同一個 COMPASS 容器映像檔,只是 code 不依賴 COMPASS,見下方 FAQ)
  train.py                               (RSL-RL PPO 訓練入口,已跑通)
  scripts/
    smoke_test.py                        (獨立驗證腳本,不經過任何訓練框架)
  zm_robot_omni_training/
    config/
      robots.py                          (zm_robot 的 ArticulationCfg)
      scene_assets.py                    (terrain/light/camera,ported 自 mobility_es,內容本來就跟 X-Mobility/COMPASS 無關)
      environments.py                    (目前只有 simple_office 一個場景,USD 資產已複製進自己的 usd/)
      zm_robot_env_cfg.py                (完全自包含的 ManagerBasedRLEnvCfg,觀測拆成 policy(MLP,1D)/camera(CNN,2D)/lidar(CNN,2D) 三組,對應 RSL-RL 的 obs_groups 慣例)
      agents/rsl_rl_ppo_cfg.py           (RSL-RL 的 CNN+MLP actor-critic + PPO 演算法設定)
    mdp/
      action/holonomic_perfect_control_action.py  (action_dim=3,已拔掉 ActionVisualizer)
      observations.py                    (lidar_occupancy_grid + camera_img_chw 等,[B,C,H,W] 格式配合 RSL-RL 的 CNNModel)
      rewards.py / termination.py / events.py / commands.py / curriculum.py  (皆為 ported,邏輯等效但零 COMPASS import)
      utils.py                           (goal_pose_in_robot_frame)
    wrapper/env_wrapper.py               (ZmRobotEnvWrapper,collision_checker 永遠回報「未初始化」= 純均勻隨機 reset,同 COMPASS 自己無 occupancy map 時的退化行為)
    usd/
      zm_robot/                          (機器人 USD)
      office/                            (場景 USD,從 COMPASS 複製,是靜態資產不是程式碼)
```

**FAQ:為什麼還在用 COMPASS 的 docker 容器跑?** 這只是「Isaac Lab 的執行環境」(CUDA/Isaac Sim/Isaac Lab 安裝好的容器映像檔),跟「COMPASS 的程式碼」是兩件事——我們不 import 任何 `mobility_es`/`compass` 的 Python 檔案,但容器裡裝的 Isaac Lab/isaaclab_rl/rsl_rl 套件本身跟 COMPASS 無關,借用同一個已經裝好環境的容器只是省去重新建置映像檔,之後如果要徹底脫離也只要另外包一個單純的 Isaac Lab 容器即可(不是本輪範圍)。

## 1. 目標

仿照 NVIDIA COMPASS 的 residual RL + distillation 框架,但針對 zm_robot 的 mecanum(全向輪)底盤,訓練一個吃多感測器輸入、輸出全向 `/cmd_vel`(vx, vy, wz)的導航政策,讓機器人真正用上側移能力,而不是像現在的 `zm_robot_compass_policy` 一樣退化成差動輪行為。

跟 `zm_robot_compass_policy`(沿用 COMPASS 的 Carter pretrained checkpoint,純視覺、非全向)是**兩個平行、不互相取代**的 policy,可以共存。

## 2. 系統總覽:兩個子系統

| 子系統 | 位置 | 性質 |
|---|---|---|
| 訓練側 | `zm_robot_omni_training`(獨立 repo,見第 0 節),**完全不依賴 COMPASS/X-Mobility**,只依賴 Isaac Lab 本身 + RSL-RL | Isaac Lab / RSL-RL / PyTorch,離線訓練,不是 ROS2 |
| 部署側 | `zm_robot_omni_policy`(本 package) | ROS2 node,吃感測器 topic,跑推論,發 `/cmd_vel` |

跟現有的 `zm_robot_compass_policy`(COMPASS Carter pretrained checkpoint)是完全獨立、不共用任何程式碼/模型的兩套東西,連訓練框架都不一樣(這邊是 Isaac Lab + RSL-RL 從零訓練,不是 COMPASS 的 residual RL + distillation)。

## 3. 感測器輸入規格

| 感測器 | Topic(對應 zm_robot 現有命名) | 用途 |
|---|---|---|
| SICK LiDAR ×2 | `/sick_lidar0/scan`, `/sick_lidar1/scan` | **進推論網路輸入**(見第 5、6 節),近距離避障、全向環境感知(RGBD 只有前方視野,LiDAR 補側後方) |
| Kinect v2 RGB-D | `/kinect_v2/color/image_raw`, `/kinect_v2/depth/image_raw` | 視覺語意理解(地面材質、動態障礙物種類辨識等 LiDAR 做不到的部分)——**不再有預訓練骨幹可用,視覺特徵要從零訓練**(見第 5 節的調整) |
| IMU | `/imu` | **只做安全層**(碰撞/異常姿態偵測),不進網路輸入 |
| Odom | `/odom` | 跟現有做法一致:提供 `base_speed`/`base_yaw_rate`,TF 提供目標相對位置 |

**已確認**:
- LiDAR ×2 的資料**進推論網路**(各自 encode 後跟 RGB/depth 特徵 concat,見第 5、6 節)——跟 IMU 不同,LiDAR 補的是 RGBD 看不到的側後方環境資訊,對導航決策是直接、非冗餘的輸入,不是只做安全層就夠。
- IMU **不進**推論網路輸入,只服務獨立的安全監控節點(見第 8 節)。理由:IMU 的角速度跟 odom 重疊,對導航決策的增量貢獻有限,但拿來做「跌倒/碰撞後緊急停止」很直接,犯不著為此把 IMU 塞進網路、拉長訓練/推論的輸入維度。

## 4. Action Space:全向(Holonomic)

輸出 3 個自由度:`vx`(前後)、`vy`(側移)、`wz`(轉向),對應到 `geometry_msgs/Twist` 的 `linear.x`、`linear.y`、`angular.z`。

**已調整**:原本沿用 COMPASS 6 維 action vector(配合 `idx_masks` 相容性)的慣例已經沒有存在理由——那是為了跟 X-Mobility 的固定輸出格式相容,現在整套網路從零寫,`action_dim` 直接簡化成 3(`vx, vy, wz`),不用再做 index masking。

新 ActionTerm:`HolonomicPerfectControlAction`

- 仿照 `NonHolonomicPerfectControlAction` 的「理想運動學直接改 root pose」寫法:把 `(vx, vy)` 在機器人本體座標系下轉到世界座標再積分更新 root 位置,`wz` 積分更新 yaw。
- **刻意不**驅動真實的 4 顆輪子 + 36 個被動滾子關節做物理模擬——PhysX 沒有內建 mecanum 滾子異向摩擦力模型,真實物理驅動容易滑動不穩、訓練難收斂。這點跟 COMPASS 自己對 Carter 的選擇一致。
- 部署到真實 zm_robot 時,這組 `(vx, vy, wz)` 直接發到 `/cmd_vel`,由 zm_robot 既有的底層馬達控制器(mecanum 逆運動學)負責換算成輪速——這層已經存在,不需要新開發。
- Isaac Sim 模擬測試時,如果場景的 Action Graph 有 `HolonomicController` 節點(NVIDIA 官方內建,Kaya 機器人範例用的那個),可以用它來驗證「真實物理驅動 mecanum 輪子」是否可行,跟 kinematic 版本的訓練結果做比對,但**訓練本身仍用理想運動學版本**。

## 5. 網路架構(Sensor Fusion)

```
RGB image ──────► [CNN encoder,從零訓練]                                 ┐
Depth image ─────► [獨立小型 CNN encoder,從零訓練]                       ├─► concat ─► 自己的 recurrent/MLP core
LiDAR ×2 ─► [合併投影成 local occupancy grid] ─► [小型 2D-CNN encoder]    │   (RSL-RL 的 ActorCritic,從零設計)
route vector ───────────────────────────────────────────────────────────┤
goal_heading ───────────────────────────────────────────────────────────┤
speed/yaw_rate ─────────────────────────────────────────────────────────┘
                                                                          │
                                                                          ▼
                                                          Action head → (vx, vy, wz)
```

**已調整(重大)**:原本「RGB backbone 凍結沿用 X-Mobility」的設計前提不成立了——不用 COMPASS/X-Mobility 之後沒有現成的視覺預訓練權重可用,RGB/Depth 的 CNN encoder 都要**從零訓練**,預期收斂速度會比原計畫慢很多,且 recurrent core(history/sample latent)原本是 X-Mobility 架構自帶的設計,現在要换成 RSL-RL 的標準 ActorCritic 架構(通常是 CNN feature → MLP,不一定需要 recurrent,先用最簡單的 feedforward MLP,除非驗證後發現真的需要時序記憶才加 RNN/GRU)。

設計原則:

1. ~~RGB backbone 凍結、沿用 X-Mobility 預訓練權重~~ —— 作廢,改成從零訓練的 CNN(見上方調整說明)。
2. **Depth 走獨立 encoder**,不跟 RGB 一起走同一個 backbone。
3. **LiDAR encoder(已確認)**:2 條 LiDAR scan 先依照 URDF 裡 `sick_lidar0_joint`/`sick_lidar1_joint` 的外參,投影合併成以機器人為中心的 **local 2D occupancy grid**(例如 ±5m、解析度 0.1m/cell),再用一個小型 2D-CNN encode 成特徵向量。
   - 選這個而不是 raw range array + 1D-CNN 的原因:occupancy grid 跟 RGB/depth 的 2D 空間特徵圖維度/語意更接近,之後如果要做跨模態的空間對齊融合(例如把視覺的障礙物偵測跟 LiDAR 的佔據格做 cross-attention)會自然很多;代價是多一層「合併兩條 scan 投影成 grid」的前處理,且 grid 的範圍/解析度/原點選擇是新增的超參數,需要在實作時另外調。
   - 兩條 LiDAR 合併成同一張 grid(而不是各自各一張再 concat),因為它們是同一個機器人上互補視野的感測器,合併後才是完整的 360° 環境資訊。
4. Action head 輸出維度從 COMPASS 原本實際用到的 2 維(vx, wz)擴充成 3 維(vx, vy, wz)。

## 6. Observation / Reward 設計(擴充現有 `env_cfg.py` 模式)

- `PolicyCfg`(actor 可見,部署時也要有):新增一個 `lidar_occupancy_grid` observation term(兩條 LiDAR 合併投影後的 local occupancy grid,見第 5 節),取代原本分開兩個 raw scan term 的設計。
- `PrivilegedCfg`(只給 critic,訓練時用,部署不需要):depth ground truth、camera_to_world、camera_intrinsics、contact forces 之類的特權資訊幫助收斂——這是 asymmetric actor-critic 的標準做法,RSL-RL 本身就支援(不是 COMPASS 特有),一樣不需要額外做 teacher-student 蒸餾兩個網路。
- Reward:`goal_reaching`(距離 reward)+ `goal_reached_reward` + `base_contact_penalty`——這幾個邏輯簡單,**自己重新實作**(見第 10 節 port 清單),**但這些本身不會自動讓政策學會用側移**——如果只是單純縮短距離,RL 很可能還是收斂成「轉向 + 前進」的非全向解法。

### 6.1 誘發側移行為的 reward shaping(已確認:4 個機制分開用,不混在一起)

COMPASS 的 `goal_heading_alignment`(原本 weight=0,被註解關閉)邏輯是獎勵「**接近目標時的最終停靠朝向**」(`distance_weight = exp(-distance/threshold)`,離目標越近權重越高),不是行進中朝向。這跟誘發側移要解決的問題是兩件不同的事,所以分成以下 4 個獨立機制,各自負責不同目的、分開調權重,全部自己重新實作(不 import COMPASS):

1. ~~新增「行進中朝向鎖定」reward~~ —— **已完成**:`mdp/rewards.py` 的 `route_heading_lock()`,weight=0.5。**實作方式跟原本設想的「貼齊 route tangent」不一樣,但等效且更乾淨**:原本的 mapless route 是每一步都從機器人「當下位置」重新算到目標的直線,所以「貼齊 route tangent」其實等於「貼齊瞬時目標方位角」——跟非全向車本來就會做的「轉身對準目標」沒有區別,不會逼出側移。改成**在 goal 指派的那一刻,把機器人當下朝向鎖起來**(`UniformCollisionFreePoseCommand.locked_heading_w`,在 `_resample_command()` 裡記錄),整個 episode 獎勵維持這個鎖定朝向、不是瞬時目標方位角——這樣橫向誤差才真的只能靠 `vy` 修正,轉身是被懲罰的。
2. ~~「到站停靠朝向」reward~~ —— **已完成**:`goal_heading_alignment()`,weight=1.0,跟 #1 用不同的 reward term、各自獨立權重。
3. ~~Action smoothness penalty~~ —— **已完成**:`action_rate_penalty`/`action_penalty` 從 weight=0.0 打開成 0.01,對稱套在 3 個 action 上,沒有另外重罰 `wz`。
4. **場景/課程設計配合**——**本輪刻意簡化,未做**:沒有另外搬 COMPASS 的貨架場景、沒有寫「目標點偏向側邊」的 curriculum。理由:`reset_base` 的起始朝向跟 `goal_pose` 的目標位置本來就是**互相獨立**取樣的,根據旋轉對稱性,機器人起始朝向跟目標方位角的夾角本來就已經接近均勻分布在 ±180°——大約一半的 episode 本來就需要顯著的側向修正才能走到目標,不靠額外課程設計也有不錯的覆蓋率。先用這個簡化版驗證 #1~#3 有沒有效果,如果觀察到側移行為太弱再回來做這項。

**風險提醒**:#1 的權重設太高,政策可能優先顧朝向、犧牲抵達效率,或在窄道裡來回震盪——用 curriculum 漸進加權重(跟現有機制一致的做法),從小權重開始調,不要一開始就設大。目前 weight=0.5 是未調校的起始值。

## 7. 訓練管線規劃(獨立於 COMPASS,RSL-RL 端到端)

沒有 imitation learning base policy、沒有 residual RL、沒有 distillation——單一機體、單一網路,從零用 PPO(RSL-RL)端到端訓練:

| 階段 | 做法 |
|---|---|
| 環境 | `zm_robot_env_cfg.py`(已完成,見第 0 節),`scripts/smoke_test.py` 驗證過可以 spawn + step。 |
| 訓練 | `config/agents/rsl_rl_ppo_cfg.py`(CNN+MLP 網路架構、PPO 超參數)+ `train.py`(`RslRlVecEnvWrapper` 包裝 `ZmRobotEnvWrapper`,跑 `OnPolicyRunner`)。**已完成,跑通 3 輪 iteration**,權重/訓練時長還沒調。 |
| 匯出 | 訓練完的 RSL-RL checkpoint(`.pt`)要自己寫轉換腳本匯出成 ONNX/TensorRT(沒有 COMPASS 的 `onnx_conversion.py`/`trt_conversion.py` 可以借,輸入/輸出格式也完全不同——多了 LiDAR occupancy grid 輸入,action 是 3 維不是 6 維)。**待寫,排在訓練跑得動之後**。 |
| 部署 | 第 8 節的 `omni_policy_node` 載入上面匯出的模型。 |

## 8. 部署側 ROS2 package 規劃(`zm_robot_omni_policy`)

跟 `zm_robot_compass_policy` 同樣的套件風格(ament_python),但這次是**兩個 node**:

### 8.1 `omni_policy_node`(推論主節點)

- 訂閱:`/sick_lidar0/scan`、`/sick_lidar1/scan`、`/kinect_v2/color/image_raw`、`/kinect_v2/depth/image_raw`、`/odom`、`/goal_pose`。**不訂閱 `/imu`**——IMU 確定不進網路輸入。
- 輸出:`/cmd_vel`,`linear.x`、`linear.y`、`angular.z` 三個都會是非零值。
- `publish_action()` 的限速邏輯沿用 `zm_robot_compass_policy` 的 `np.clip` 模式,新增 `max_linear_y`/`min_linear_y` 限制參數。

### 8.2 `imu_safety_monitor_node`(獨立安全層,新)

- 訂閱:`/imu`。
- 偵測邏輯:
  - **異常姿態/跌落**:roll/pitch 超過閾值(例如 > 30°)判定側翻風險。
  - **異常衝擊**:線加速度的高頻變化量(或直接看 raw linear_acceleration 扣除重力後的量級)超過閾值判定碰撞。
- 觸發後的動作:發布一個 `/emergency_stop`(`std_msgs/Bool`)topic,`omni_policy_node` 訂閱這個 topic,一旦為 true 就不論推論結果,強制發布 zero-velocity 的 `/cmd_vel` 並停止後續推論,直到外部清除旗標。
- 刻意跟推論節點**分成兩個 process**,不是同一個 node 裡的分支邏輯——安全層要能獨立於推論模型是否崩潰/卡住都持續運作,這是安全關鍵邏輯跟 AI 推論邏輯分離的基本原則。

與 `zm_robot_compass_policy` 的關係:`omni_policy_node` 跟 `zm_robot_compass_policy` 的推論節點是平行可替換的,用 launch 參數選擇要跑哪一個,兩者都共用 `zm_robot_navigation` 的 `map_tf_broadcaster`;`imu_safety_monitor_node` 則是跟 policy 無關的通用安全層,之後兩個 policy node 都可以共用同一個安全監控節點。

## 9. 風險與待決策項(需要在動工前拍板)

1. ~~LiDAR encoder 表示方式:raw range array vs occupancy grid~~——**已確認:合併投影成 local occupancy grid + 2D-CNN**(見第 5 節)。grid 範圍/解析度/原點等超參數留待實作時調。
2. ~~IMU 要不要進網路輸入,還是只做安全層~~——**已確認:只做安全層**,獨立 `imu_safety_monitor_node`(見第 8.2 節)。
3. ~~Reward shaping 是否足以誘發側移行為~~——**已確認:4 個獨立機制**(行進中朝向鎖定 reward、既有 `goal_heading_alignment`、action smoothness penalty、場景/curriculum 配合),見第 6.1 節。實際權重數值仍要留到訓練階段調。
4. **PhysX 模擬 mecanum 輪子的穩定性**——確認理想運動學(kinematic)訓練版本跟實際部署時真實輪子動力學之間的 sim-to-real gap 有多大,是否需要在 reward 裡加入動作平滑度限制來降低這個差距。
5. ~~訓練計算資源~~ —— **已實測**:RTX 5070 Ti 16GB 單卡,`num_envs=6` 可以跑(150 輪 ≈14 分鐘),`num_envs=8` 直接 OOM(`torch.OutOfMemoryError`,14.65GB 已用)。Camera(320×512 RGB)+ 2 個 LiDAR CNN 分支讓每個 env 的記憶體開銷遠比純狀態 RL 任務高,**不是線性隨 `num_envs` 增加**——4→6 還撐得住,6→8 就爆,之後真要衝更大規模需要降低 camera 解析度或減少 CNN hidden_dims 才有空間。端到端 RL 收斂速度仍比原計畫慢,目前只跑過 500 輪量級。
6. **安全層觸發閾值**(roll/pitch 角度、加速度門檻)要配合 zm_robot 實際的機構極限跟感測器 noise floor 校正,目前只是佔位數字。
7. ~~是否依賴 COMPASS/X-Mobility 程式碼或模型~~——**已確認並完成:完全不依賴**。`zm_robot_omni_training` 零 `import mobility_es`/`import compass`,訓練演算法用 RSL-RL(Isaac Lab 官方配套,非 COMPASS)。

## 10. 分階段實作計畫

1. ~~新增 `HolonomicPerfectControlAction`、`zm_robot` 的 `ArticulationCfg`、`zm_robot_env_cfg.py`,`--num_envs 1` smoke test~~ —— **已完成**。
2. ~~補 LiDAR occupancy grid 的 observation term~~ —— **已完成**:`lidar_occupancy_grid()`,兩條 `MultiMeshRayCasterCfg`(見第 5 步)合併投影成 100×100 occupancy grid。
3. ~~完全移除 COMPASS/X-Mobility 依賴~~ —— **已完成**:`mdp/{utils,rewards,termination,events,commands,curriculum,observations}.py`、`wrapper/env_wrapper.py`、`config/{scene_assets,environments,zm_robot_env_cfg}.py` 全部重寫/port 完,零 COMPASS import。獨立的 `scripts/smoke_test.py` 實測跑通(spawn、100 步、所有 Manager 正確註冊),action_dim 簡化成 3(vx, vy, wz)。`ActionVisualizer` 確認是純 debug 視覺化、無功能性副作用,整個移除。
4. ~~RSL-RL 訓練迴圈~~ —— **已完成**:`config/agents/rsl_rl_ppo_cfg.py`(CNN+MLP 的 `RslRlCNNModelCfg`/PPO 設定)+ `train.py`,實際跑了 3 輪 learning iteration(`--num_envs 1`),mean reward/metrics 正常記錄無崩潰。網路架構直接做了 CNN(第 5 節的目標架構,不是先用拉平 MLP 的簡化版)——`camera`/`lidar` 各自獨立 observation group(`[B,C,H,W]`),對應 RSL-RL 的 `obs_groups` 慣例,比原計畫的「先 MLP 再疊 CNN」更直接做到位。目前是 symmetric actor-critic(critic 還沒接 privileged 特權資訊,見第 6 節)。
   **過程中修掉的 3 個真實 bug**(非本專案邏輯問題,是環境/API 版本問題):(a) `RayCasterData.ray_hits_w`/camera RGB 輸出的 dtype/shape 問題(uint8→float、channels-last→first);(b) `isaaclab_rl` 的 `stochastic` 已棄用欄位沒呼叫 `handle_deprecated_rsl_rl_cfg()` 清理,直接傳給新版 `rsl-rl-lib` 的 `CNNModel` 建構子會噴 `TypeError`;(c) 容器裡 `python`(base image 自帶、`rsl-rl-lib 5.0.1`)跟 `python3`(Isaac Lab 自己釘的環境)是兩份不同安裝,bare `python` 會解析到不相容的版本——**跑這個 repo 一定要用 `python3`,不要用 `python`**。
5. ~~解決「LiDAR 只量到地板」的限制~~ —— **已完成**:原因是 Isaac Lab 的基礎 `RayCasterCfg` 架構上**只支援一個 mesh prim**(`ray_caster.py` 原始碼明確寫死這個限制),而且它的 mesh cache 是跨所有環境共用、只認一個固定路徑的單一全域 mesh——這正是為什麼官方範例全部只打 `/World/ground`(真正全域、不隨 env 複製的東西),沒有任何「打場景家具」的先例可抄。換成 `MultiMeshRayCasterCfg`(接受多個 `RaycastTargetCfg`,支援 `{ENV_REGEX_NS}` per-env 模板、`merge_prim_meshes=True` 合併場景底下所有子網格)解決。實測驗證:10 步後 occupancy grid 有 132 個非零格(滿格 10000),raw ray hit point 落在 z≈0.297(牆面高度,不是地板),180 條 ray 裡 111 條真的打到東西——確認是真的偵測到 office 場景的牆/家具,不是空的。重跑一次 RSL-RL 訓練(3 轮 iteration)確認整條管線仍然正常。
6. ~~小規模訓練驗證 reward 曲線合理、沒有數值爆炸~~ —— **已完成**(`--num_envs 4 --max_iterations 50`):
   - **抓到一個真的 bug 並修掉**:`base_contact` 幾乎每個 episode 一開始就觸發,導致 reward 曲線完全沒意義。直接追蹤 contact force 數值才查出來——`HolonomicPerfectControlAction` 每步用 kinematic teleport 硬改 root pose,PhysX 在 reset 後第 1、2 步會把這個「瞬間跳位」誤判成穿透,算出瞬間破百萬牛頓的虛假接觸力(實測:step1 ≈1,003,481 N、step2 ≈129,645 N、step3 起歸零),完全不是真的碰撞。修法:新增 `termination.py` 的 `illegal_contact_after_grace()`,reset 後頭 5 步不檢查 `base_contact`,包著 isaaclab 原生的 `mdp.illegal_contact`。修完 reward 範圍從原本亂飄的 -100~-611 收斂成穩定的 -100~-123,episode 長度從 32 步穩定成長到 140~171 步。
   - **數值穩定性確認**:全程 50 輪 `Mean value loss`(0.27~2220)、`Mean surrogate loss`(-0.04~0.01)、`Mean entropy loss`(穩定 4.27 附近)都在合理範圍,`nan_pose` 全程 0——沒有數值爆炸。
   - **`base_contact` 仍是主要終止原因(佔 75~99%)**——這是預期中的正常現象,不是 bug:LiDAR 現在真的看到 office 場景的牆/家具(第 5 步的修正成果),但政策才訓練 50 輪、CNN 完全從零開始,還沒學會避障,撞牆是正常的早期行為。`goal_reached` 全程 0 次也是同理,還太早。真正看「學得動不動」要等規模化訓練(第 7 步)。
7. ~~reward 調整 + 驗證側移行為是否真的出現~~ —— **已完成,結果正面**:實作第 6.1 節機制 1~3(見上方),跑了 150 輪訓練(`--num_envs 4`,約 14 分鐘),載入訓練完的 checkpoint(`model_149.pt`)、關掉隨機探索用 deterministic policy 跑 60 步,直接量測三個 action 的統計量:

   | | mean | std | 平均絕對值 |
   |---|---|---|---|
   | vx(前進) | 0.2984 | 2.1065 | 1.6195 |
   | **vy(側移)** | -0.4564 | 2.5400 | **1.9469** |
   | wz(轉向) | -0.2425 | 0.6441 | 0.5119 |

   **`vy` 的平均絕對值比 `vx` 還大**,連續多步的樣本(`[1.21, 1.52, 1.57, 1.32, 1.36]`)也是穩定同號、不是雜訊亂跳——`route_heading_lock` 這個機制確實成功逼出側移行為,不是只有理論上說得通,是訓練 150 輪就能實測到。

   訓練過程本身也健康:`Mean episode length` 從 4 步穩定成長到 149.67 步,`error_heading` 降到 0.2328、`error_pos_2d` 降到 0.9584(已經很接近 goal_reached 的 0.5 門檁),但 150 輪還太少,`goal_reached` 全程仍是 0 次——這代表方向對、但要真的學會導航還需要大規模訓練(第 8 步)。第 6.1 節刻意跳過的機制 4(場景/curriculum)目前看來不是必要的,暫不補。
8. **規模化訓練(`num_envs=6 --max_iterations 500`,約 50 分鐘)—— 做了,結果有正面證據但也抓到一個新問題,還沒解決**:
   - **正面證據:機器人真的穩定學會到達目標過**——iteration 95~137 這 43 輪之間,`goal_reached` 持續維持在 0.1667(6 個環境裡固定有 1 個成功到達),不是曇花一現的單次僥倖,證明整條 pipeline(reward 設計、CNN 架構、holonomic action space)是真的有能力產生成功的導航行為。
   - **問題:這個成功沒有被留住**——iteration 137 之後成功率就掉回 0,到 500 輪結束時 `base_contact` 回到 100%(所有 episode 都撞牆結束),比中段表現還差。
   - **已查到的根本原因**:追蹤整整 500 輪的 `Mean action std`,數值從頭到尾死死卡在 1.00~1.01,**完全沒有下降過**。PPO 的 Gaussian 策略的 std 本來是可學習參數,理論上政策越有信心應該會自己收斂變小——它卡住不動,代表政策從頭到尾都在用近乎相同幅度的隨機探索蓋掉已經學到的行為,學到的好策略會被接下來的高變異探索洗掉,這解釋了「學會又忘記」的現象。
   - **第一次修法(`init_std` 1.0→0.5、`entropy_coef` 0.005→0.001)——無效**:跑了 300 輪驗證,`Mean action std` 還是整場死死卡在 0.49~0.51(只是卡在不同起點),`goal_reached` 成功次數反而從 48 次掉到只有 1 次,Mean reward 還從 -101 惡化到 -465。證明 entropy bonus 不是主因。
   - **真正根因:直接讀 tensorboard 的 `Loss/learning_rate` 數值查出來的**——設定的是 `learning_rate=1.0e-3`,但 `schedule="adaptive"` 這個機制幾乎整場把實際 LR 壓在 **1e-5**(比設定值小 100 倍),只偶爾短暫跳到 1.5e-5~7.6e-5 又馬上壓回去。根本不是 std 單獨的問題,是整個網路的梯度更新幾乎停滯——小 batch(144 樣本)容易讓單次更新的 KL divergence 超過 `desired_kl=0.01`,adaptive 排程就把 LR 壓到底,之後再也沒機會恢復。
   - **第二次修法:`schedule` 改成 `"fixed"`**(拿掉會把 LR 壓到趨近零的機制)。同時嘗試把 `num_steps_per_env` 從 24 調到 48 想順便擴大 batch,但**直接 OOM**——PPO 的 learning phase 要對整批儲存的影像重新做 CNN forward+backward,記憶體隨 `num_envs×num_steps_per_env` 陡增,不是單純存儲 raw 影像那麼便宜,所以退回 24。
   - **第二次修法的結果:根因確認是對的,但方向不對**——跑到 104/300 輪時中途檢查:`Mean action std` 真的開始動了(0.50→0.58,不再死板),證明 LR 凍結的診斷正確。但是方向錯了(std 往上升,不是我們要的下降),而且 `Mean reward` 持續惡化(-101→-923)。判斷是 LR 解除節流後,`entropy_coef=0.001` 的「獎勵高 entropy」梯度壓力現在真的有效果、把 std 往上推,且 `learning_rate=1.0e-3` 在沒有 adaptive 保護下對這麼小的 batch 太激進——**中途停掉這次訓練**(直接 `docker.sh down` 重開容器,比在 container 內找 PID 乾淨)。
   - **第三次修法:`entropy_coef` 歸零、`learning_rate` 降到 `1.0e-4`——矯枉過正,換成「太冷」**:用 Monitor 工具每 25 輪主動追蹤,201 輪全程 reward 卡死在 -161~-167、std 死板在 0.50、**整整 201 輪 0 次 goal_reached**——不是發散,是完全停滯,`entropy_coef=0` 加上太小的 LR 讓政策沒有任何探索動力。主動停掉這次訓練(沒等跑完 300 輪)。
   - **第四次修法:取中間值——結果仍是發散,只是變慢**——`learning_rate` 1e-4→**3e-4**(介於「1e-3 太兇」跟「1e-4 太冷」之間)、`entropy_coef` 0→**0.0005**(前次 0.001 把 std 推錯方向,這次減半)。5 輪 sanity check 正常,300 輪驗證跑在背景用 Monitor 每 25 輪主動追蹤。iter 26 一度看起來有改善(reward -162→-152),iter 51 跳到 -247/-265 時先判斷可能是雜訊、沒有馬上出手,但到 **iter 76 確認是真的發散,不是雜訊**:reward 從 iter 47 的 -155 持續惡化到 -247→-313→-349→-368,連續 25 輪沒有回穩。主動停掉(`docker.sh down`),`nvidia-smi` 確認 GPU 乾淨。
   - **四次 bracketing 的結論**:`lr=1e-3`(無論 adaptive 或 fixed)都發散、`lr=1e-4`完全停滯、中間的 `lr=3e-4` 還是發散(只是比 1e-3 慢)——**在這個 144 樣本(`num_envs=6 × num_steps_per_env=24`)的 batch 大小下,找不到一個同時「學得動」又「不發散」的 learning_rate**。這跟使用者一開始提出的懷疑方向一致:問題可能不是沒調對參數,而是 batch 本身對 PPO 這種 on-policy 演算法來說太小太吵,調 LR 只是在「太冷」跟「太兇」之間换位置,中間地帶可能根本不存在或極窄。
   - **第五次修法嘗試 A:調大有效 batch(放棄,硬體卡死)**——想試 `num_mini_batches` 4→1 或 4→2(梯度更新吃更大的 shard,直接降低梯度估計雜訊,不動 `num_envs`/`num_steps_per_env`)。結果兩個值都**立即 OOM**——light-check 時 GPU 在 rollout 收集階段就已經用到 14.4~14.53/15.46 GiB,learning phase 對更大 shard 的影像重新做 CNN forward+backward 完全沒有餘裕。證實 `num_mini_batches=4` 已經是這張 GPU 上的硬性下限,沒有「調大 batch 降噪」這條路可走。
   - **第五次修法嘗試 B(使用者建議:換成「會自動調學習率」的演算法,而不是再手動 bracket 固定 LR)**——回頭讀 RSL-RL `algorithms/ppo.py` 的 adaptive schedule 原始邏輯:`kl_mean > desired_kl*2` 就把 LR 砍成 `/1.5`(下限 1e-5),`kl_mean < desired_kl/2` 就 `*1.5`(上限 1e-2)。原始 `desired_kl=0.01` 代表門檁只要 0.02,而這組 144 樣本的小 batch 幾乎每次更新都會超過這個門檁,所以 LR 直接被壓到 1e-5 的底,再也沒機會回升(跟第一次修法量到的現象完全吻合)。這次把 `desired_kl` 放寬到 **0.05**(門檁變 0.10)、`learning_rate` 重新設回 **1e-3**(讓機制自己去找合適的值,不是我們預設死),`entropy_coef=0.0003`,`num_mini_batches` 維持安全值 4。5 輪 sanity check 正常,300 輪跑在背景。
   - **第五次修法 B 的結果:也發散,而且抓到更直接的根本證據**——監控到 iter 162/300(訓練時間 15 分 24 秒)時主動停止:`Mean reward` 從 iter 0 的 -101.91 持續惡化(iter25≈-128、iter50≈-143、iter75≈-158、iter100≈-223、iter125≈-245)一路掉到 iter150~162 附近穩定卡在 **-437~-441**,`Metrics/goal_pose/error_pos_2d` 來到 5.4552(比第 8 步「學會到達目標」那段的 0.9584 差了 5 倍以上)、`error_heading` 0.9502、`base_contact` 終止率 83%、`goal_reached` 全程 0 次——不是短暫雜訊,是連續 160 輪的單向惡化後卡在一個「到處亂走、最後撞牆」的壞策略。直接讀這次的 tensorboard `Loss/learning_rate`(用 `event_accumulator` 從 container 內讀):LR 完全沒有穩定下來,在 **1e-5(下限)跟 ~2.9e-3 之間劇烈跳動**(例:iter0→1e-5、iter20→5.8e-4、iter70→1.3e-3、iter100→1e-5、iter120→2.9e-3、iter150→8.6e-4、iter170→2.25e-5),代表 KL-based adaptive 機制本身量到的 KL 雜訊太大、太不穩定,門檁放寬也沒讓它穩定在某個值,而是讓政策一直在「幾乎沒更新」跟「更新幅度是另一輪的 ~300 倍」之間反覆橫跳——這種跳動本身就足以讓訓練發散,跟固定 LR 的發散是不同機制、但結果一樣壞。已停止訓練(`docker.sh down`),GPU 確認乾淨。
   - **五次實驗的總結論**:固定 LR(1e-3 太兇、1e-4 太冷、3e-4 中間值仍發散)跟 adaptive LR(無論 `desired_kl` 鎖緊或放寬)**在這個 144 樣本的小 batch 下全部失敗**,而且 adaptive 機制在雜訊夠大時連「自動找到合適 LR」這件事本身都做不到(LR 本身劇烈震盪)。這已經不是單一槓桿沒調對的問題——五次嘗試涵蓋了 fixed/adaptive 兩種排程、LR 橫跨 1e-4~1e-3 整個合理範圍、以及 entropy_coef 0~0.001,結果全部指向同一個瓶頸:**`num_envs=6`(GPU 記憶體硬上限)讓每次 PPO 更新只有 144 筆樣本,這對一個從零訓練的視覺+LiDAR CNN 政策來說雜訊太大,不管怎麼調 LR 排程都救不回來**。
   - **後備方案(使用者已提出,現在應該認真評估)**:**PPO 這種 on-policy 演算法在 `num_envs=6` 這麼小的平行環境數下本質上太沒效率**——每次更新後舊資料就丟棄,通常需要大量平行環境的樣本量才穩,而這台 GPU 的記憶體上限讓這條路走不通。下一步應該認真考慮換成 SAC 之類的 off-policy 演算法(有 replay buffer 可以跨多個 iteration 重複利用過去收集的資料,等效把樣本量撐大,不受 `num_envs` 卡死;環境數少但資料可以重複學習,理論上比 PPO 更適合這個硬體限制)。這是 RSL-RL 以外的另一套函式庫/訓練迴圈,工程量不小(需要另外的 replay buffer、off-policy 的 actor-critic 更新邏輯,CNN+LiDAR 的觀測架構可以沿用),建議作為下一步的正式決定方向,而不是再继续在 PPO 的超參數空間裡做第六次 bracket。
   - **改用 SAC(Stable-Baselines3,`train_sac.py` + `config/agents/sb3_sac_cfg.py`)**:`MultiInputPolicy`(影像走 NatureCNN、向量走 flatten)、`buffer_size=5000`(replay buffer 以 uint8 影像存放,受主機 RAM 限制)、`ent_coef="auto"`、動作空間設為實際速限 `[±0.6, ±0.6, ±1.57]`(SAC 必須有界動作空間,Isaac Lab 預設的 ±inf / ±100 不能用)。PPO 與 SAC 用不同的 `ObservationsCfg`(SAC 版 `SACObservationsCfg`:單一 `policy` 群組、`concatenate_terms=False`、影像保持 0~255)。長時間訓練要用 `docker exec -d` 啟動——從使用者 session 起的 `nohup &` 在登出時會被連同容器一起殺掉(exit 137)。
   - **SAC 25,000 步:訓練穩定、沒發散,但沒學會導航**。用基準線比對才看出來(`scripts/eval_sac.py --zero/--random`)。
   - **評估/訓練環境的三個缺陷(皆已量測確認、已修正)**:
     1. **每個環境的第一個 episode 一開始指令是全零**(Isaac Lab 只在 `step()` 尾端呼叫 `command_manager.compute()`,`reset()` 不會),第一步 `goal_reached` 就拿「目標=機器人所在位置」判定成功。→ `ZmRobotEnvWrapper.reset()` 覆寫,reset 後補算指令與觀測。這個缺陷讓小樣本評估被灌水(例如「不動」策略 57% goal_reached)。
     2. **`collision_checker` 是空殼,23% 的出生點(240 次中 55 次)落在家具裡**,一出生就有 2e5~6e5 N 接觸力、第 5 步(寬限期結束)判 `base_contact`。→ 新增 `OccupancyMapCollisionChecker`(讀 `usd/office/omap/occupancy_map.yaml`,距離場膨脹 0.35 m;對 55 個實測撞點零漏報,0.3 m 漏 1 個)。**注意 checker 必須在 `super().__init__` 之前建立**,因為 goal command 在自己的 `__init__` 就把它存起來了。修正後出生即撞牆 23% → 0.4%(1/240)。目標點也同步受此檢查,並新增 `min_goal_distance=1.0` 硬下限。
     3. **評估步數不足**:一個 episode = 100 s = 1000 步,之前的 800 步評估永遠看不到 `time_out`。評估改為 2500 步。
   - **修正後的基準線比對(6 envs、2500 步)**:

     | 策略 | episodes | goal_reached | base_contact | time_out | 平均長度 | 結束時距目標 |
     |---|---|---|---|---|---|---|
     | 完全不動 | 12 | 0% | 0% | 100% | 1000 | 3.84 m |
     | 隨機動作 | 45 | 0% | 96% | 4% | 284 | 4.05 m |
     | SAC (model_final) | 16 | 0% | 50% | 50% | 776 | 6.96 m |

     結論:SAC 輸出近乎零的動作(平均 |a|≈0.02~0.03,上限 0.6/1.57),**沒有朝目標前進**(結束時距離 6.96 m 比「不動」的 3.84 m 還遠,50% 是慢慢飄去撞牆)。reward 為每步約 -1、撞牆 -1000、到達 +2000,在探索不足時「站著不動」是局部最佳解;`ent_coef` 從 0.93 一路降到 0.35,探索壓力持續下降。**之前 PPO 實驗中「連續 N 輪 goal_reached」的結果,也可能含有缺陷 1 的假成功,不可全信。**
   - **SAC 調參記錄(2026-11-05,皆在修正後的環境上,6~16 個平行環境)**——判斷依據一律用「確定性評估 + 完全不動/隨機基準線」,不用訓練中的 ep_rew:
     | 實驗 | 觀測 | 改動 | 結果 |
     |---|---|---|---|
     | 2 | 影像+LiDAR 占據圖+向量(CNN) | reward 改成進步量(`goal_progress`,權重 100)、`target_entropy=-1`、`gradient_steps=2` | 3.8 萬步 0/103 到達;`ent_coef` 塌到 0.027;評估結束距離 6.99 m(比不動還遠),政策朝固定方向漂移 |
     | 3 | 只有向量(MLP,16 envs,~100 步/秒) | 同上,拿掉影像以隔離「CNN 學不動」與「任務學不動」 | 15 萬步:撞牆 80→15 次但到達只有零星幾次,學成「慢慢晃、少撞牆」 |
     | 4 | 向量 | 目標範圍 ±5→±3 m(最近 1.5 m)、到達朝向容許 0.1→0.5 rad、新增 `progress_m` 指標 | **首次穩定學會**:前進量 +1.5 m,8~9 萬步到達率 ~55%;但沒有 LiDAR,撞牆約 45% |
     | 5 | 向量+36 扇區 LiDAR 最近距離(`lidar_polar_ranges`) | 觀測加 LiDAR | 撞牆降到 ~1~2 次/萬步;10 萬步評估:44% 停在目標 0.5 m 內,但朝向誤差<0.5 rad 只有 1/25,成功 0~8% |
     | 6 | 同 5 | `route_heading_lock` 乘 `1-exp(-d/1.0)`,近目標淡出(原本與 `goal_heading_alignment` 衝突:轉 Δ 的淨利益 ≈ 0.74Δ−0.5Δ²,Δ>1.5 rad 為負) | 6 萬步評估:68% 停在 0.5 m 內,朝向<0.5 rad 4/25,成功 2/25——位置改善,仍不轉身(平均朝向誤差 1.53 rad ≈ 隨機水準 π/2) |
     | 7 | 同 5 | `goal_heading_alignment` 權重 1.0→4.0 | 6 萬步評估(34 episodes):**成功 11/34(32%)**、朝向<0.5 rad 14/34、平均朝向誤差 1.28 rad;代價:停在 0.5 m 內降為 44%、出現 15% 撞牆、平均結束距離 1.11 m(樣本小,尚不能斷定是權重的副作用) |
   - **評估方法修正(重要)**:小樣本(25~35 個 episode)的成功率雜訊可達十幾個百分點——同一個 checkpoint(實驗 7 的 6 萬步)兩次評估是 32%(11/34)與 19%(5/27)。此後一律用大樣本(16 envs × 6200 步,100+ 個 episode)並報 95% Wilson 信賴區間,**只有區間不重疊才說有差異**(`scripts/eval_sac.py` 已內建;另輸出結束距離、朝向誤差、距離×朝向聯合合格數與逐項獎勵)。訓練中的隨機策略統計(`[TERM]` 行)與確定性評估會有很大落差(實驗 7 延長訓練:訓練中 33%、評估 10%),不可互相取代。
   - **實驗 7 延長訓練(接續 6 萬步 → 15 萬步,空 replay buffer + 2000 步暖機)**:大樣本評估成功率 **32.8%**(95% CI 25.5~41.2,n=134)→ **10.3%**(5.8~17.5,n=107),確實變差。逐項獎勵顯示 15 萬步學會轉身(朝向懲罰 -201→-68)卻停在離目標更遠處(結束距離 1.08→1.75 m),且平均總獎勵反而較高(-114 對 -175)——**獎勵變高、成功率變低**:原本的 `goal_heading_alignment`(每步 `-4·|朝向誤差|·exp(-距離)`)讓「待在遠處」更划算,且單一 episode 內它的總和(-190~-270)遠大於進步獎勵(+13)。
   - **實驗 8(加速度限制 0.8 m/s² / 1.57 rad/s²)**:6 萬步 2/27、15 萬步 0/27 成功;**樣本太小,且在補上信賴區間之前做的,不能據此判斷加速度限制的影響**。使用者隨後要求改回 2.0 / 3.14(程式已改回,註解有記錄),實驗 8 的 checkpoint 與實驗 2~7 不可比較。部署到實機若要用較低加速度,與模擬的落差仍待處理。
   - **實驗 9(唯一改動:`goal_heading_alignment` → `goal_heading_progress`)**:朝向對準從每步懲罰改成進步量獎勵(前一步 |朝向誤差| − 這一步 |誤差|),在離目標 1 m 內線性淡出、權重 30;其餘與實驗 7 完全相同,從頭訓練 15 萬步。大樣本評估:

     | checkpoint | 成功率(95% CI) | n | 停在 0.5 m 內 | 朝向 <0.5 rad | 距離×朝向 | 平均總獎勵 |
     |---|---|---|---|---|---|---|
     | 實驗 7,6 萬步 | 32.8%(25.5~41.2) | 134 | 64/134 | 55/134 | 45/134 | -175 |
     | 實驗 7 延長,15 萬步 | 10.3%(5.8~17.5) | 107 | 17/107 | 58/107 | 12/107 | -114 |
     | 實驗 9,6 萬步 | 5.1%(2.4~10.7) | 117 | 49/117 | 14/117 | 6/117 | -29 |
     | **實驗 9,15 萬步** | **50.3%(43.1~57.5)** | **181** | **156/181** | **115/181** | **110/181** | **+95** |

     實驗 9 的 15 萬步與實驗 7 的 6 萬步信賴區間**勉強不重疊**(43.1 > 41.2),與實驗 7 的 15 萬步則明顯分開。實驗 9 隨訓練變好(5%→50%),實驗 7 隨訓練變差(33%→10%);且實驗 9 的總獎勵與成功率方向一致(成功組 +219、超時組 -21),不再是「獎勵高、成功率低」。**限制:每種設定只跑了一次訓練,沒有多個隨機種子,無法把差異完全歸因於獎勵改動而排除 run-to-run 變異。**
   - **種子重跑(實驗 9 設定,`--seed` 固定,15 萬步,同一評估協定;腳本 `scripts/run_seed_check.sh`)**:

     | 訓練 | 成功率(95% CI) | n | 撞牆 | 停在 0.5 m 內 | 朝向 <0.5 rad | 距離×朝向 | 平均總獎勵 |
     |---|---|---|---|---|---|---|---|
     | 原始(未固定種子) | 50.3%(43.1~57.5) | 181 | 4% | 156/181 | 115/181 | 110/181 | +95 |
     | 種子 101 | 23.0%(16.7~30.7) | 135 | 9% | 84/135 | 40/135 | 32/135 | +9 |
     | 種子 202 | 41.5%(33.8~49.6) | 147 | 2% | 114/147 | 78/147 | 72/147 | +67 |

     **同一設定的三次訓練,成功率從 23% 到 50%,相差約 27 個百分點**;101 與 202 的信賴區間不重疊(30.7 < 33.8),101 與原始也不重疊,202 與原始重疊。**結論:單次訓練的成績不能代表設定**,此前所有「某改動有效/無效」的判斷(含實驗 7 的 32.8%→10.3%、實驗 9 的 50.3%)都只有一次訓練支撐,需打折。101 與 202 的差別主要在朝向與撞牆(朝向合格 30% vs 53%;撞牆 9% vs 2%),靠近目標的程度相近(平均進步量 +1.51 vs +1.60 m)。
   - **混淆因素(我先前漏掉)**:實驗 7 的「15 萬步」(10.3%)是**接續訓練**得到的(從 6 萬步 checkpoint 接續,replay buffer 為空、2000 步隨機暖機),不是從頭跑 15 萬步;實驗 9 的 15 萬步是從頭訓練。兩者**不是同條件比較**,「朝向進步量優於朝向懲罰」尚未被乾淨驗證。能確定的只有:實驗 9 設定從頭訓練 15 萬步的三次結果為 23%、41.5%、50.3%(平均約 38%);缺少「實驗 7 設定從頭訓練 15 萬步、多個種子」的對照。
   - **乾淨對照:朝向獎勵形式 × 種子(皆從頭訓練 15 萬步、同評估協定;`scripts/run_penalty_control.sh`,`--reward penalty`)**

     | 朝向獎勵 | 種子 | 成功率(95% CI) | n | 停在 0.5 m 內 | 朝向 <0.5 rad | 平均進步量 |
     |---|---|---|---|---|---|---|
     | 進步量(實驗 9) | 101 | 23.0%(16.7~30.7) | 135 | 62% | 30% | +1.51 m |
     | | 202 | 41.5%(33.8~49.6) | 147 | 78% | 53% | +1.60 m |
     | | 未固定 | 50.3%(43.1~57.5) | 181 | 86% | 64% | +1.91 m |
     | 懲罰(實驗 7 獎勵) | 101 | 14.2%(8.9~21.8) | 113 | 28% | 69% | +1.16 m |
     | | 202 | 12.3%(7.5~19.6) | 114 | 18% | 64% | +1.20 m |

     解讀:(1) 成對看,種子 101 兩組信賴區間重疊(無法區分),種子 202 不重疊(進步量版較高)。(2) 進步量版 3 次(23/41.5/50.3%)皆高於懲罰版 2 次(14.2/12.3%),但 5 次訓練中恰好是進步量版排前三的排列機率約 10%(單邊),**統計上不顯著**。(3) 懲罰版兩個種子結果很接近(穩定偏低),進步量版種子間變異很大。(4) **失敗型態兩個種子都重現**:懲罰版朝向好(64~69%)卻停不到目標附近(18~28% 在 0.5 m 內);進步量版位置好、朝向較差——與逐項獎勵的推測一致(懲罰項把策略推向先轉朝向)。**先前「進步量版把成功率從 10% 拉到 50%」是誇大**;乾淨比較的差距約 13% 對 23~50%。
   - **證據等級(前一版,2~3 個種子)**:朝向進步量獎勵「很可能」較好,信心為中等。
   - **五個種子的完整比較(種子 101/202/303/404/505,皆從頭訓練 15 萬步,評估協定同上;`scripts/run_more_seeds.sh` 跑 303/404/505;未固定種子的 50.3% 那次不計入統計)**

     | 種子 | 進步量版(實驗 9) | 懲罰版(實驗 7 獎勵) |
     |---|---|---|
     | 101 | 31/135 = 23.0% | 16/113 = 14.2% |
     | 202 | 61/147 = 41.5% | 14/114 = 12.3% |
     | 303 | 28/134 = 20.9% | 33/130 = 25.4% |
     | 404 | 110/200 = 55.0% | 36/132 = 27.3% |
     | 505 | 93/192 = 48.4% | 41/136 = 30.1% |
     | 種子平均(SD) | **37.8%(15.2)** | **21.8%(8.1)** |
     | 合併 episode | 323/808 = 40.0% | 140/625 = 22.4% |

     統計:平均差 +15.9 個百分點;以種子為單位的置換檢定(252 種排列)雙尾 **p = 0.087**,Welch t = 2.06,bootstrap 95% CI [+2.4, +29.1] 個百分點;進步量版在 5 個種子中勝 4 個(303 是懲罰版略高,區間重疊)。**解讀:方向一致、效果量中等(約 16 個百分點),但以 α=0.05 衡量尚未顯著**(此置換檢定的雙尾 p 值最小可到 2/252 ≈ 0.008,所以 0.087 不是檢定本身的下限造成的;若改用成對的符號翻轉檢定,5 對的雙尾下限是 0.0625)。進步量版種子間變異大(21~55%),懲罰版較集中(12~30%)。失敗型態在 5 個種子都重現:懲罰版朝向好(朝向 <0.5 rad 約 64~92%)但位置差(停在 0.5 m 內約 18~40%),進步量版位置好、朝向較差。
   - **評估異常(尚未查明)**:`exp9_seed505` 的評估中有 5 個 episode 的結束距離為 141~152 m(其餘最大僅 4.4 m),使平均結束距離變成 4.43 m、平均進步量 −1.99 m;全部是 time_out,所以成功率不受影響。其他 9 次評估沒有出現。推測為模擬/物理異常(例如機器人被彈出或掉出地圖),**未驗證**;這是運動學直接寫入 root pose 的環境,值得查是否有出界保護。
   - **混合獎勵(`--reward hybrid`,`scripts/run_hybrid.sh`)**:進步量項(權重 30,不變)+ 縮小的懲罰項(`goal_heading_alignment` 權重 1.0,原懲罰版為 4.0);僅此一個變數,obs 同為 state_lidar,種子 101~505,從頭 15 萬步,同評估協定。

     | 種子 | 進步量版 | 懲罰版 | 混合版 |
     |---|---|---|---|
     | 101 | 23.0% | 14.2% | 38.5%(57/148) |
     | 202 | 41.5% | 12.3% | 9.2%(10/109) |
     | 303 | 20.9% | 25.4% | 42.5%(77/181) |
     | 404 | 55.0% | 27.3% | 40.6%(63/155) |
     | 505 | 48.4% | 30.1% | 45.0%(76/169) |
     | 平均(SD) | 37.8%(15.2) | 21.8%(8.1) | **35.2%(14.7)** |
     | 合併 episode | 323/808 = 40.0% | 140/625 = 22.4% | 283/762 = 37.1% |

     統計(以種子為單位的置換檢定,252 種排列):混合−進步量 = −2.6 pp,p = 0.83,bootstrap 95% CI [−20.0, +13.2],混合勝 2/5;混合−懲罰 = +13.3 pp,p = 0.135,CI [−0.8, +24.9],勝 4/5;進步量−懲罰 = +15.9 pp,p = 0.087(同上)。**結論:混合獎勵沒有勝過進步量版(統計上無法區分,點估計還略低);它比懲罰版高但未達顯著。** 混合版 5 個種子中 4 個落在 38~45%,只有種子 202(9.2%:到位 40/109 但朝向誤差平均 1.42 rad,`goal_heading_alignment` 累積 −61)是離群,所以「較穩定」只是觀察,證據弱(一個離群值就決定 SD)。權重 1.0 為一次性選擇,未調過(0.5、2.0 等未試)。
   - **X-Mobility 整合(尚未有訓練結果)**:模型程式、gin 設定與 state_dict-only 權重(482 MB)已放入 `zm_robot_omni_training/third_party/x_mobility/`(`scripts/extract_xmobility_weights.py` 可重新產生權重),`models/xmobility.py` 為凍結推論包裝(離線建構 DINOv2、strict 載入;分批 4 個環境推論,因為編碼器同時保留 12 層注意力圖,16 環境整批會 OOM)。重要事實:**X-Mobility 只吃 RGB**(無深度輸入),輸出 6 維 twist 但只有 vx、wz 有意義(以差速車訓練);深度改走我們自己的 CNN 分支。環境:`--obs state_lidar_xmob`(+ `--reward {progress,penalty,hybrid}`),觀察 = 原 83 維 + X-Mobility 2051 維 + 深度圖 1×64×104;回放緩衝區上限 10 萬(~3 GB)。**目前只通過 60 步冒煙測試與分批等價測試;X-Mobility 單獨開車的基準評估(`scripts/eval_xmobility_base.py`)與任何 X-Mobility 版訓練都還沒有結果。**
   - **證據等級(更新)**:「進步量版較好」為**中等偏弱**證據(4/5 種子、p≈0.09,未達顯著);尚不足以當論文主張。要更有把握需再加種子,或改用更大的效果(例如先解決位置/朝向取捨後再比較)。
   - **目前最佳 checkpoint**:`logs/sb3/zm_robot_omni_sac_state_lidar/2026-11-06_14-34-16/model_150000_steps.zip`(實驗 9,成功率 50.3%)。**仍是簡化任務**:目標 ±3 m(最近 1.5 m)、朝向容許 0.5 rad、向量+LiDAR 觀測(無影像)、加速度 2.0/3.14,與部署需求(±5 m、朝向 0.1 rad、影像、實機加速度)有差距。
   - **剩餘失敗**:45% 超時、4% 撞牆。距離×朝向合格的 110 個 episode 中只有 91 個判成功,**推測**其餘約 19 個是到位但速度未降到 <0.1(旋轉動作 |wz| 平均 0.475 偏高),未直接量測速度,待驗證。
   - **工具**:`scripts/record_video.py`(俯視相機疊圖錄 mp4;相機須朝下,`--pitch_sign -1` 才有畫面,+1 會拍到天空全黑)、`scripts/diag_spawn.py`、`scripts/spawn_test.py`、`scripts/check_lidar_obs.py`。技能目錄調查:Isaac Lab 自帶 `isaaclab-debugging-rl-training` 等技能(位於 `IsaacLab-Arena/submodules/IsaacLab/skills/user/`),其「reward 不等於成功指標、一次只改一個變數、選 checkpoint 看任務指標」的原則已實際套用在上述診斷。
   - **工具/流程備忘**:`train_sac.py --obs {full,state,state_lidar}`;訓練每 1000 步印 `[TERM]`(到達/撞牆/超時次數 + `progress_m`);`scripts/eval_sac.py --obs ... --checkpoint ...` 輸出結束距離、朝向誤差、距離×朝向聯合合格數;**LiDAR 版訓練時不要同時開第二個模擬**(曾因此在 `MultiMeshRayCaster` 初始化時出現 CUDA illegal memory access,原因為推測)。
   - **已知限制**:單次評估只有 25~34 個 episode,成功率差幾個百分點屬雜訊等級;SB3 的 `ep_rew_mean` 是滾動平均,對 reward 權重改動不可跨實驗比較。
   - **下一步(待決定)**:(a) 調查 `exp9_seed505` 評估中結束距離 141~152 m 的 5 個 episode(出界/物理異常);(a') 是否再加種子(目前 5 對、p≈0.09);(b) 診斷剩餘超時(量測結束時的線/角速度;「距離×朝向合格但未判成功」的 episode 數:原始 19、101 為 1、202 為 11);(c) 解決位置/朝向的取捨(目前兩種獎勵各偏一邊);(d) 逐步恢復目標範圍 ±5 m、朝向容許 0.1 rad;(e) 加回影像。
9. ~~自己寫 ONNX/TensorRT 匯出腳本~~ —— **已完成(2026-11-10)**,見第 11 節。
10. ~~這個 package(`zm_robot_omni_policy`)補上 `omni_policy_node` 程式碼,接訓練好的模型~~ —— **已完成(2026-11-10)**,見第 11 節;只做過重播驗證,尚未閉迴路實測。
11. `imu_safety_monitor_node` —— 2026-11-10 實作,2026-11-11 因誤觸發移除,見第 11 節;要重做需先校正門檻。

## 11. 部署現況(2026-11-10)

**模型。** `最終指令 = clip(X-Mobility 基底 [vx, 0, wz] + SAC 殘差)`,殘差範圍為完整動作範圍的 0.5 倍。訓練結果(種子 101,各一次訓練):進步量 80.4%、懲罰 85.3%、混合 82.9%;X-Mobility 單獨 14.0%。預設部署混合版(`actor:=hybrid`),三個 actor 都已匯出。

**匯出。** `zm_robot_omni_training/scripts/export_deploy.py`(容器內、純 PyTorch)匯出兩個 ONNX:`xmobility.onnx`(凍結 X-Mobility 單步,含 RSSM 狀態進出)與 `sac_residual_actor_<獎勵>.onnx`(SAC actor,確定性,已換算成實際單位)。ONNX 對 PyTorch 的最大差距約 1e-5 以下。TensorRT 引擎用 `scripts/build_engines.sh` 在本機建立(FP32)。

**ROS2 package。** `omni_policy_node`:訂閱 RGB、深度、兩顆 LiDAR、`/odom`、`/goal_pose`、`/emergency_stop`,用 TF 取得目標與 LiDAR 相對 `base_link` 的位置,發布 `/cmd_vel`(linear.x、linear.y、angular.z)。前處理(`preprocess.py`)是訓練端觀測項的 numpy 版。`sync_mode:=timer` 以 10 Hz 執行(實機);`sync_mode:=stamp` 每個感測器時間戳執行一次(模擬/重播)。`imu_safety_monitor_node`(第 8.2 節)在 2026-11-10 實作過,2026-11-11 **已移除**:門檻(傾斜 30°、衝擊 10 m/s²)沒有校正,在 Isaac Sim 裡太容易誤觸發急停。策略節點仍訂閱 `/emergency_stop`,可接外部安全裝置,但這個 package 裡沒有節點發布它。要重做的話,需先用實機或 Isaac Sim 的 IMU 資料校正門檻。

**Isaac Sim 首次閉迴路測試發現的問題(2026-11-10)。** 用 `zm_robot_cartographer_omni_policy.launch.py` 在 Isaac Sim 導航時,收到目標後 0.4 s,當時還在的 IMU 安全監控就鎖住緊急停止(衝擊 11.3 m/s² > 10)。原因有二:(1) 部署節點直接送出階梯式速度指令(0 → 0.6 m/s、-1.57 rad/s),而訓練環境在指令與機器人之間有 2.0 m/s²、3.14 rad/s² 的加速度限制;(2) 三軸的衝擊判斷把麥克納姆滾子造成的垂直震動(9.2 m/s²)也算進去。修正:`omni_policy_node` 的 `/cmd_vel` 以 50 Hz 依同樣的加速度上限逼近策略指令(到達目標時也是減速到零後才停止發布;緊急停止與感測器逾時仍立刻歸零),衝擊判斷改成只看水平方向。之後安全監控仍太容易觸發,已整個移除(見上);加速度限制保留,它只用重播測過,**尚未在 Isaac Sim 重新驗證**。

**驗證(重播,開迴路)。** `scripts/record_deploy_ref.py` 在訓練環境錄下 156 步(3 個回合)的原始感測器資料與 PyTorch 策略的輸出;`replay_validation_node` 把它們當成 ROS2 topic 逐步發布,比對節點的 `/cmd_vel`。

| 項目 | 結果 |
| --- | --- |
| 最終指令對 PyTorch(147 個參考一致的步) | 最大差 0.0031、平均 0.0004(m/s 或 rad/s);通過門檻 0.02 |
| 全部 156 步 | 最大差 0.114(都在下述 9 個排除步內) |
| 觀測(目標位置、朝向、LiDAR 36 扇區、速度) | 與模擬器觀測差 ≤ 1e-6;深度圖只有個別像素差 1/255 |
| 引擎層級(不經 ROS,三個 actor) | actor 引擎對 PyTorch 殘差最大差 ≤ 0.0004;X-Mobility 引擎基底指令最大差 0.0009 |
| 推論時間(前處理 + 兩個引擎,RTX 5070 Ti) | 平均 10.1 ms、p95 12.7 ms |
| `timer` 模式冒煙測試 | 指令頻率 9.8 Hz;感測器停止後輸出零速度 |

**驗證過程發現的訓練環境問題(未修,會影響訓練資料的一小部分)。**

1. **reset 後第一步的 LiDAR 觀測是舊的。** 回合重置把機器人瞬移後,LiDAR 還停在重置前的位置(錄到的感測器位置離機器人 4~7 m),所以每個回合第一步的 36 扇區觀測是錯的。約每回合 1 步(平均回合 64~108 步)。
2. **整次執行的第一步,X-Mobility 拿到全零的路線。** 第一次 reset 時觀測先在目標指令算出前計算一次並被快取。只影響每個環境的第一個回合第一步。
3. **LiDAR 射線方向固定在世界座標**,不隨機器人轉(`ray_alignment="yaw"` 沒有生效)。因為每顆都是 360° 掃描且觀測在機體座標重算,對 36 扇區觀測沒有影響;但訓練用的是「每顆 360°、看不到車身」的 LiDAR,實機 SICK 是 270° 且會被車身遮擋。

上述 1、2 讓參考輸出在這些步不可信,所以重播比對時排除這 3 步與各自之後 2 步(前一步指令與 RSSM 狀態的延續影響),共 9 步,並在報告中分開列出。

**尚未驗證。**

- 閉迴路:沒有在 Isaac Sim + ROS2 bridge 或實機上跑過,重播的感測器資料不會因節點的指令而改變。
- 實機感測器落差:Kinect v2 彩色 1920×1080(水平視角約 84°,規格值、未量測)會被直接縮成 512×320(訓練是 92.6°、同一相機出 RGB 與深度);Kinect 深度相機的視角與彩色不同;SICK 的視角、雜訊、車身遮擋都與訓練不同。
- 實機動力學:訓練用理想運動學,沒有輪子打滑與加速度落差。
- 任務範圍:訓練目標在 ±3 m 內、到位容許 0.5 m / 0.5 rad。

