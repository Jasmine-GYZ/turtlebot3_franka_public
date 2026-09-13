# 任务导航 — 关键配置（已核实）

`patrol_task.py` 的巡逻配置，这些坐标/yaw/顺序是用户在 Gazebo 里目视核实过的，不要重新推理改动。

## 初始位姿
- `INITIAL_X = -5.30`, `INITIAL_Y = -0.50`, `INITIAL_YAW = 0.0`
- 与 `turtlebot3_franka.launch.py` 的 spawn 参数一致（robot spawn 在 `-x -5.30 -y -0.50 -Y 0.0`）。
- 该位置在**左墙（x≈-5.05）西侧、门洞缺口**处（左墙 Wall_16 顶端 y=-1.03 与 Wall_18 底端 y=0.016 之间约 1m 缺口）。机器人从这里穿门进入客厅。

## 访问顺序（3 → 1 → 0 → 2）
```python
WAYPOINTS = [
    ("table_3", -4.7, -3.1, 0.0),           # 面朝 +X（living_room_table_3）
    ("table_1", -3.3, -3.3, - math.pi / 2), # 面朝 -Y（living_room_table_1）
    ("table_0", -2.1, -1.2, math.pi / 2),   # 面朝 +Y（living_room_table_0）
    ("table_2", -0.6, -2.6, 0.0),           # 面朝 +X（living_room_table_2）
]
```
注意：table_1 的 yaw 是 **-π/2（面朝 -Y）**，不是 +π/2。

## 串行导航流程（每个点位一个独立 NavigateToPose action）
```
发目标 N → NavigateToPose action 执行 → 结果回调 _on_arrived
   ├─ status=4 (SUCCEEDED)：到达 → 站稳 → _do_detection 扫描 → waypoint_idx+1 → 发目标 N+1
   └─ status=6 (ABORTED)：没到达 → 记 warning → waypoint_idx+1 → 发目标 N+1
```
- status=4 = 到达（正常终止）；status=6 = 没到达（规划/跟踪失败或超时）。
- "站稳等扫描"只发生在 status=4 之后，不是 status=6 的触发原因。

## 桌子实际位置（example.world，供对照）
| 模型 | pose |
|---|---|
| living_room_table_0 | (-2.1, 0) |
| living_room_table_1 | (-3.3, -2.1) |
| living_room_table_2 | (0.63, -2.6) |
| living_room_table_3 | (-3.5, -3.1) |
观察点距桌面约 1.2m。

## 地图/坐标系约定
- `map.yaml` 的 `origin: [-5.62, -4.35]` **只是把地图图像对齐到世界**，不是坐标偏移。
- 因此 map 帧 = Gazebo world 帧，世界坐标可直接当 map 帧坐标发布，无需转换。

## 任务流程
到达点位 → 站稳 → 视觉识别（`_do_detection`）→ 识别完成才移动到下一点位。串行执行。

## 视觉识别（GroundingDINO + SAM2）

`_do_detection()` 已接入真实视觉流水线 `vision_pipeline.py`：

1. 订阅相机话题 `/camera/image_raw`，缓存最新一帧（另有深度 `/camera/depth/image_raw`、内参 `/camera/camera_info`）。
2. GroundingDINO 检测（三类物品 prompt，`ITEM_NAMES`）→ NMS + 置信度过滤 → SAM2 框提示分割。
3. 计算每个实例掩码的像素中心点，归类到三种物品之一（`classify_phrase`）。
4. 像素中心 + 对齐深度反投影 + TF（相机光学帧→map）解算出 `/map` 帧 3D 坐标。
5. 三类物品计数写入 `item_counts`，并在 RViz 的 `/map` 下以 Marker（话题 `/detected_items`）标记位置。
6. 扫描全部完成后，在终端打印三种物品英文名称与数量。
7. 可选保存标注图到 `~/turtlebot3_detections/`。

注意事项：

- 模型（GroundingDINO + SAM2）在**首次识别时**加载，第一个观察点会额外花几十秒。
- 必须用虚拟环境跑节点（同时含 torch/groundingdino-py/sam2 与 rclpy），先 `source ~/vision_env/bin/activate`。
- 刻意不用 cv_bridge（当前环境 numpy 2.x 与 ROS 的 cv_bridge ABI 不兼容），图像解码用手动 numpy。
- 相机光学帧：Gazebo 给相机帧加 `turtlebot3/` 前缀，启动文件已用 `override_frame_id: camera_rgb_optical_frame` 覆盖，与 TF 树对齐。
- 去重：相邻观察点可能扫到同一物体（或把远处物体认错），按 `/map` 坐标距离合并（`DEDUP_DIST=0.25m`），同一位置只计一次、只标一个 Marker（首次登记为准）。
- 单个观察点内：先 NMS，再按框中心距离合并同一物体的重复框（只留得分最高者），同一物体不会框两个框。
- 空桌不框选：低于桌面高度（`MIN_OBJECT_Z=0.70m`，/map z）的检测判为误检丢弃。
