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
到达点位 → 站稳 → 视觉识别扫描（`_do_detection`，目前是 stub）→ 扫描完成才移动到下一点位。串行执行。

## 视觉识别（占位符，待实现）

`_do_detection()` 目前是**占位符（stub）**：

- 只执行 `time.sleep(2.0)`，然后打印 `🔍 扫描中...` 和 `✓ 扫描完成`，**没有真正拍照/识别**。
- `detection_results` 一直是空 `{}`（最后 `_all_done()` 会把它打印出来）。

后续实现：把 `sleep` 换成真正的「订阅图像 → 检测 → 去重计数」逻辑，
并把结果写入 `detection_results`。图像处理节点之后再写。
