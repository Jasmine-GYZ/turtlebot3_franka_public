# TurtleBot3 + FR3 导航仿真

TurtleBot3 Waffle Pi + FR3 机械臂在 **Gazebo Fortress (Ignition Gazebo)** 中的仿真包，
包含机器人仿真、仿真场景、导航（Nav2）、串行巡逻任务节点、一套
GroundingDINO + SAM2（+ INSID3 复核）的开放集视觉识别流水线，
以及基于 **MTC（MoveIt Task Constructor）** 的抓取阶段。

## 包含内容

| 包 / 目录 | 说明 |
|---|---|
| `franka_description` | FR3 机械臂 URDF/meshes（tag 2.8.1），仿真 URDF 的硬依赖 |
| `turtlebot3_manipulation_gazebo` | 仿真 spawn 启动（`turtlebot3_franka.launch.py`）、TB3+FR3 URDF、ros2_control 配置、网格 |
| `turtlebot3_manipulation_navigation2` | Nav2 启动（`navigation.launch.py`）、地图/参数、任务节点 `patrol_task.py`、视觉识别流水线 `vision_pipeline.py`、INSID3 复核 `insid3_review.py`、抓取阶段 `grasp_phase.py` + 调试入口 `dining_grasp_task.py`、cmd_vel 桥接 |
| `turtlebot3_manipulation_grasp` | **MTC 抓取包**：抓取规划节点 `grasp_node`、`/detect_grasp_target` 视觉服务 `detect_grasp_target_node.py`、抓取服务 `grasp_service.launch.py`、目标物配置 `config/objects.yaml`。⚠️ 该视觉服务**默认不在本包启动**，而是由 `patrol_task` 同进程托管（见「运行」） |
| `turtlebot3_moveit_config` | **MoveIt 2 配置**：SRDF、运动学/关节限位/控制器/OMPL 参数，`move_group` 启动（抓取规划依赖） |
| `wpr_simulation_ros2` | 仿真场景资源（`worlds/example.world` + `models/`） |
| `third_party/` | 视觉识别用的第三方源码（git submodule，固定 commit） |
| `docs/` | 两份说明文档（见下） |

## 目录结构

```
.
├── franka_description/                 # FR3 机械臂 URDF/meshes（依赖）
├── turtlebot3_manipulation_gazebo/     # 仿真包
├── turtlebot3_manipulation_navigation2/  # 导航 + 任务 + 视觉识别包
├── turtlebot3_manipulation_grasp/      # MTC 抓取包（规划节点 + 抓取服务）
├── turtlebot3_moveit_config/           # MoveIt 2 配置（SRDF/运动学/OMPL）
├── wpr_simulation_ros2/                # 仿真场景包
├── third_party/                        # git submodule（固定 commit）
│   ├── sam2/      # facebookresearch/sam2
│   ├── INSID3/    # visinf/INSID3
│   └── dinov3/    # facebookresearch/dinov3
├── setup.sh                            # 一键搭建环境
├── download_weights.sh                 # 下载模型权重
├── build.sh                            # 干净环境构建
├── requirements.txt                    # 视觉识别 Python 依赖
├── docs/
│   ├── turtlebot3-fr3-fortress-integration.md  # TB3+FR3 集成细节（含改动清单）
│   └── task-navigation.md              # 任务节点关键配置（位姿/顺序/流程）
└── README.md
```

## 环境要求

- **Ubuntu 22.04 + ROS 2 Humble**
- **Gazebo Fortress (Ignition Gazebo)** + `ros_gz_sim` / `ros_gz_bridge` / `gz_ros2_control`
- **Navigation2**：`ros-humble-navigation2`、`ros-humble-nav2-bringup`
- **MoveIt 2 2.5.10 + MTC**：抓取阶段依赖，见下方「额外依赖」
- **Python 3.10**（虚拟环境，见下）

## 快速开始（一键）

```bash
# 1. 克隆（含第三方 submodule）
cd ~/turtlebot3_ws/src
git clone --recursive https://github.com/Jasmine-GYZ/turtlebot3_franka_public.git

# 2. 搭建环境（ROS 包 + submodule + 虚拟环境 + 权重）
cd turtlebot3_franka
bash setup.sh              # 无 GPU 用 CPU 版 torch；有 NVIDIA GPU 加 --cuda

# 3. 构建
cd ~/turtlebot3_ws
bash src/turtlebot3_franka/build.sh   # 或见下方「构建」一节的 colcon 命令
source install/setup.bash
```

> 若 clone 时忘了 `--recursive`，在仓库根补一次 `git submodule update --init --recursive`。

## 额外依赖（apt 安装即可，`setup.sh` 已自动安装）

| 依赖 | 安装 |
|---|---|
| `xacro`、`robot_state_publisher`、`joint_state_publisher_gui` | `sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui` |
| `ros2_control`、`ros2_controllers`、`gripper_controllers` | `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers` |
| `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control` | `sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control` |
| `rviz2` | `sudo apt install ros-humble-rviz2` |
| **MoveIt 2 + MTC**（抓取阶段必需） | `sudo apt install ros-humble-moveit ros-humble-moveit-configs-utils ros-humble-moveit-task-constructor-{core,capabilities,msgs,visualization}` |

> `franka_description` 已打包在本仓库里，无需再单独 clone。
>
> ⚠️ **MoveIt 版本必须与 MTC 匹配**：apt 仓库的 `ros-humble-moveit-task-constructor-core`
> 只有 `0.1.3` 一个版本，它是针对 **MoveIt 2.5.10** 编译的。若本机是 2.5.9，
> `libmoveit_*.so.2.5.9` 与 `libmoveit_*.so.2.5.10` 对不上，`grasp_node`
> 链接期和运行期都会失败（`ldd` 一片 `not found`）。升级：
>
> ```bash
> sudo apt install --only-upgrade 'ros-humble-moveit-*'
> ```
>
> 升级后若构建报「没有规则可制作目标 `libmoveit_move_group_interface.so.2.5.9`」，
> 是旧的 build 缓存作祟，删掉重编即可：`rm -rf build/ install/turtlebot3_manipulation_grasp`

## 视觉识别（GroundingDINO + SAM2 + INSID3 复核）

巡逻节点到达观察点后调用 `vision_pipeline.py` 做开放集检测 + 闭集复核 + 实例分割：

```
相机帧 → GroundingDINO 检测（原始 logits 逐框 argmax 取单一标签，消除多标签拼接）
       → NMS + 置信度过滤
       → INSID3 闭集复核（冻结 DINOv3 骨干，剔除干扰物 / 修正误判标签）
       → SAM2 框提示分割 → 掩码像素中心 + 深度反投影 → /map 3D 坐标
       → 目标物品计数 + RViz Marker 标记
```

### 依赖

- **第三方源码（submodule）**：`third_party/` 下的 `sam2`、`INSID3`、`dinov3`，
  已用 git submodule 固定 commit，无需再 clone；GroundingDINO 用 PyPI 的 `groundingdino-py`
  包（模型代码 + config 都来自它），无需额外源码仓库。
- **模型权重**（见 `download_weights.sh`）：
  - GroundingDINO / SAM2（公开 URL，自动下载到 `$MODEL_WEIGHTS_DIR`，默认 `~/model_weights`）：
    - `$MODEL_WEIGHTS_DIR/groundingdino/groundingdino_swint_ogc.pth`
    - `$MODEL_WEIGHTS_DIR/sam2/sam2_hiera_small.pt`
  - INSID3 用 DINOv3 base 骨干（约 342MB，官方门控，需手动下载，放在仓库内）：
    - `turtlebot3_manipulation_navigation2/scripts/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth`
- **INSID3 参考图**：`turtlebot3_manipulation_navigation2/scripts/reference_views/` 下 18 类物体、
  每类 6 视角（前/后/左/右/上/下）的 Gazebo 渲染截图，用作闭集复核的类别原型。
- **Python 虚拟环境**：`setup.sh` 会创建（默认 `~/vision_env`，可用 `VISION_ENV_DIR` 改），
  含 torch、groundingdino-py、sam2、supervision、hydra，以及 INSID3 依赖 einops/scikit-learn、
  rclpy/sensor_msgs 等。运行任务节点前必须先 `source ~/vision_env/bin/activate`。

### 关键说明

- 相机 RGB 话题：`/camera/image_raw`，深度 `/camera/depth/image_raw`，内参 `/camera/camera_info`。
- 待计数物品 / 阈值在 `vision_pipeline.py` 顶部配置区（`ITEM_NAMES`、`BOX_THRESHOLD` 等）。
- 闭集复核在 `insid3_review.py`：用冻结的 INSID3（DINOv3 骨干 + 位置偏置去相关，Train-Free 只推理零训练）
  对每个候选框 crop 与各类参考原型做余弦相似度 argmax；命中目标类则保留并修正标签，命中干扰类或低置信则丢弃。
  类别集合与参考图路径集中在文件顶部 `INSID3_CLASSES` / `REF_MODELS_ROOT` 配置区；权重缺失时
  `vision_pipeline.py` 会捕获异常并降级为「无复核」继续运行。
- 四种物品（`apple` / `coke can` / `bowl` / `banana`）计数结果：扫描完成后在终端打印，同时累计在 `item_counts`。
- 识别到的物品在 RViz 的 `/map` 坐标系下以 `visualization_msgs/Marker`（话题 `/detected_items`）标记；
  位置由「像素中心 + 对齐深度反投影 + TF 相机光学帧→map」解算得到。
- 相邻观察点可能扫到同一物体（或把远处物体认错），按 `/map` 坐标去重（`DEDUP_DIST` 阈值），
  同一位置只计一次、只标一个 Marker，以**首次登记**为准（后续重复检测直接丢弃）。
- 单个观察点内同一物体不会被框两次：GroundingDINO 输出的框先 NMS，再按**框中心距离**合并
  同一物体的重复框（只留得分最高者）。
- 空桌不框选：低于桌面高度（`MIN_OBJECT_Z=0.70m`，/map z）的检测判为桌腿/地面等结构，直接丢弃。
- 识别结果（数量 + 每个目标像素中心 + /map 3D 位置）汇总在 `detection_results`，标注图存到 `~/turtlebot3_detections/`。

### 路径 / 配置（环境变量）

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `MODEL_WEIGHTS_DIR` | `~/model_weights` | GroundingDINO / SAM2 权重根目录 |
| `VISION_ENV_DIR` | `~/vision_env` | 虚拟环境目录（setup.sh 用） |
| `ANSWER_OUTPUT_DIR` | `~/turtlebot3_ws/submissions` | 答案 JSON 输出目录 |

## 构建

把本仓库放到你自己的 `turtlebot3_ws/src/` 下（克隆或软链均可），然后：

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install \
  --packages-select franka_description \
  turtlebot3_manipulation_gazebo \
  turtlebot3_manipulation_navigation2 \
  turtlebot3_manipulation_grasp \
  turtlebot3_moveit_config \
  wpr_simulation_ros2
source install/setup.bash
```

> 用 `--symlink-install`，之后改 `patrol_task.py` 等 Python 脚本无需重新编译。
> 仓库根的 `build.sh`（克隆后位于 `<ws>/src/turtlebot3_franka/build.sh`）
> 会在干净环境里构建（清空 ROS 前缀、只 source 系统 ROS），避免脏终端污染。
>
> ⚠️ **推荐直接用 `bash build.sh`，不要手敲上面的 colcon 命令**。除了清环境，
> `build.sh` 还内置了一处必需的环境修补：`turtlebot3_manipulation_grasp` 带
> msg/srv，rosidl 生成 Python 类型时 `ament_cmake_python` 会调用用户级的
> setuptools，而它需要比系统 `packaging 21.3` 更新的版本，直接报
> `TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'`。
> `build.sh` 会往 `<ws>/.build_pydeps` 装一份隔离的 `packaging`，只通过
> `PYTHONPATH` 注入本次构建（不碰系统、不碰 `~/.local`；删该目录即回退）。
>
> 构建时出现 `--allow-overriding turtlebot3_manipulation_gazebo
> turtlebot3_manipulation_navigation2` 警告属**已知遗留**（apt 里另装了一份同名包），
> 不影响构建结果。
>
> 改过 `param/*.yaml` 或 `urdf/*.xacro` 后**必须重新构建**：yaml 在 Nav2 启动时读取、
> xacro 在 launch 时展开，只改源码不会生效。

## 运行（四个终端）

**顺序不能乱**：终端 1 起完再起 2，抓取服务（终端 3）必须在任务节点（终端 4）
进入抓取阶段之前就绪，否则抓取阶段会因为服务端不存在而失败。

> ★ **视觉模型只加载一份**（2026-09-16 改）：`/detect_grasp_target` 这个视觉服务
> 现在由**终端 4 的 `patrol_task` 进程内托管**，直接复用 patrol 自己那套
> GroundingDINO + SAM2 + INSID3，所以终端 3 **不再起第二个视觉进程**。
> 原因：两个进程各加载一整套模型合占 ~5 GiB / 8.15 GiB，实测双双 CUDA OOM，
> Phase 2 抓取直接报废。现在全系统只有**一个** python 进程占 GPU（~2.8 GiB），
> 用 `nvidia-smi --query-compute-apps=pid,used_memory --format=csv` 可核对。

**终端 1 — 仿真 + spawn 机器人（含 FR3 臂）**

```bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
```

启动后 Gazebo 窗口出现、机器人落在起点。确认 `/clock` 有在走（`ros2 topic hz /clock`）。

**终端 2 — 导航（Nav2 + 地图 + 参数）**

```bash
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py
```

确认代价地图起来了、AMCL 收到激光（`ros2 topic hz /scan`）。

**终端 3 — 抓取服务（MoveIt move_group + MTC 抓取节点）**

```bash
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
```

**不需要** `source ~/vision_env/bin/activate`，也**不加载任何视觉模型** ——
这里只起 `move_group` 和 MTC 抓取节点 `grasp_node`。视觉服务 `/detect_grasp_target`
由终端 4 的 patrol 进程托管（见上方说明），本 launch 的 `enable_vision` 默认 `false`。

**终端 4 — 任务节点（初始定位 → 串行巡逻 + 视觉识别 → 抓取）**

```bash
source ~/vision_env/bin/activate
ros2 run turtlebot3_manipulation_navigation2 patrol_task.py
```

> ⚠️ 虚拟环境必须是**装全依赖的那一个**（默认 `~/vision_env`）—— 全局唯一的那份
> 视觉模型就在这里加载。缺 `einops` / `scikit-learn` 时 INSID3 闭集复核会
> **静默降级为「无复核」**，香蕉这类物体可能被 GroundingDINO 误判成苹果
> —— 不报错，但结果错。启动日志里应有 `视觉模型加载完成。`
> （装了复核权重时约 77 s）与 `grasp 视觉服务已在本进程托管（共享同一套模型）`。

任务节点分两个阶段：

- **Phase 1 巡逻**：依次导航到 4 个客厅观察点（顺序 `table_3 → table_1 → table_0 → table_2`），
  到达后站稳识别（GroundingDINO + SAM2 + INSID3 复核），再移动到下一个。详见 `docs/task-navigation.md`。
- **Phase 2 抓取**：巡逻结束后（写完答案 JSON）转入抓取阶段 —— 到餐桌 3 的**观察位**选目标 →
  导航到**站位**（物体正前方 0.33~0.39 m）→ **相对闭环微调**把物体对正到
  `base_footprint(0.33, 0)` → 调 `/grasp_fixed_object` 由 MTC 执行抓取。

> 抓取阶段只读**视觉**，不读 Gazebo 真值（规则书禁止）。桌号不硬编码，
> 来自 `turtlebot3_manipulation_grasp/config/grasp_params.yaml` 的 `support_surface`；
> 可抓目标物在 `config/objects.yaml`。
>
> **当前状态（2026-09-16）**：整条链已跑通到 `stage=0`（观察位选目标 → 导航站位 →
> 相对闭环微调 → MTC 抓取服务调用全部走通），但**物理夹持尚未成功** ——
> 现象是夹爪看着包住物体、能闭合，物体却不动。判据见下。

### 只调抓取阶段（跳过巡逻）

调试抓取时不必每次跑完整巡逻，用调试入口单独起抓取阶段。

这条路径**没有 patrol 进程**，所以视觉服务没人托管，必须在终端 3 单独拉起来
（这时它才自己加载一套模型，是唯一还需要视觉 venv 的场景）：

```bash
# 终端 3（改用这个，多带 enable_vision:=true；这条路径才需要激活视觉 venv）
source ~/vision_env/bin/activate
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py enable_vision:=true

# 终端 4
source ~/vision_env/bin/activate
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py
```

常用开关：

| 开关 | 作用 |
|---|---|
| `--classes "tomato_soup_can"` | 只找指定类别（逗号分隔），用于干净对照 |
| `--skip-nav` | 跳过导航（机器人已手动停好），只测服务链路 |
| `--no-creep` | 跳过相对闭环微调 |
| `--keep-pose` | 不重发 `/initialpose`，沿用当前 AMCL 位姿（仿真已在跑、车已在桌边时重测用）|
| `--observation "x,y,yaw"` | 手工指定观察位（默认按餐桌桌沿法线自动算）|
| `--park-x/--park-y/--park-yaw` | 固定站位（默认由选中物体位置算）|

### 抓取是否夹住的判据

`grasp_phase.py` 会直接量**两指真实间距**（`fr3_leftfinger` 与 `fr3_rightfinger`
两个 TF 帧的距离），并在合爪后自动判定，认准这三行之一：

| 日志 | 含义 |
|---|---|
| `✓ 手指停在 X mm ≈ 物体窄边 Y mm → 夹到了物体（接触即停）` | **夹到了** —— 手指被物体挡住 ✓ |
| `✗ 手指停在 X mm —— 比物体窄边 Y mm 还宽 → 没夹到物体（或夹到了别的东西）` | 两指之间是空的 ✗ ⇒ 问题在抓取点的**位置/高度**，不要再查别的 |
| `? 手指合到 X mm，比物体窄边 Y mm 还小 → 物体被挤走 / 没在两指之间` | 合过头了，物体被推走 ✗ |

另外三行是有用的旁证：

- `合爪前两指真实间距 = X mm；指尖平面 base(x,y,z)` —— 合爪**前**的开口
- `两指真实间距(TF): 起始 X mm → 最小 Y mm` —— 夹持过程中的**最小**开口；
  它就是上面判据用的那个数，并附带一条自洽检查
  `✓ 与 joint1+joint2 自洽`（TF 实测间隙 应 ≈ 两关节位移之和）
- `指尖轨迹(本次调用): x a→b（最远 c） z d→e` —— **最远 x 应 ≈ 抓取点的 x** ✓

> 判据需要物体的真实窄边（来自 `config/objects.yaml`），所以日志里会同时打出
> `预期合爪: 物体窄边 Y m − 干涉 Z → 关节 Q（两指间距 R mm）` 作为参照。
>
> ⚠️ **判据用的是 TF 实测间隙，不是 `2×joint1`**（2026-09-18 改）。弃用 mimic 后
> 两指独立驱动、**可以不对称** —— 实测一次成功抓取是 joint1=0.0285、joint2=0.0372
> （和 = 0.0657 m ≈ 罐头窄边 0.0660 m），而 `2×0.0285` 只有 57.0 mm。
> 旧版拿它去比 66.0 mm，于是**每次成功都误报** `? 物体被挤走` + `✗ 两指不对称` ✗

### 为抓取调整过的导航参数

`turtlebot3_manipulation_navigation2/param/turtlebot3_use_sim_time.yaml`
（`navigation.launch.py` 加载的那份）**目前只改了 1 个值**：

| 参数 | 值 | 为什么 |
|---|---|---|
| `robot_radius` | 0.28（原 0.35） | 抓取站位要求车心离桌沿 0.26 m（`grasp_phase.py` 的 `EDGE_CLEARANCE`）。0.35 的致命膨胀区会把站位吞掉，规划器直接拒收目标点 → 车到不了桌边 |

曾一并放宽过的另外三项（`xy_goal_tolerance` 0.4、`inflation_radius` 0.45、
`cost_scaling_factor` 4.0）**已回退成原值 0.25 / 0.55 / 3.0** —— 它们只是配合性的
放宽/少绕路，回退不影响抓取可达性。

> ⚠️ 即便是 `robot_radius`，它也**同时作用于 Phase 1 巡逻**（会贴障碍物更近）。
> 动过之后巡逻阶段需要单独回归验证。

### 向前伸臂时车体前倾（✅ 2026-09-18 已修）

底盘的支撑多边形原本只有 `[x=-0.177, x=0]` —— **两个万向轮都在后方**
（`caster_back_left/right`，world 里 x=-0.177），驱动轮在 x=0，**前方没有任何支撑点**。
而机械臂 19.6 kg、质心高 0.87 m（底盘才 1.55 kg），所以向前伸臂时整车必然前倾。

**它与抓取的关系（这是 20 cm 级偏差的真根因）**：`base_footprint` 与底盘刚性固连，
底盘一低头，**在 base 系里"正确"的预计算抓取点在世界里被抬高并前移**。
视觉测量发生在 arm home（pitch≈0），而 MTC 执行发生在 arm 伸出（pitch 11.7°）——
**测量位姿 ≠ 执行位姿**，所以视觉再准也没用。

- 实测：命令 base(0.432, −0.125, 0.895)，pitch 11.55° 时执行 → TCP 落到世界 y=1.8038，
  而罐子在 y=2.0000 → **误差 0.196 m，几乎全在前后方向**
- **修法（已采纳）**：加前万向轮 `caster_front_joint`（origin `xyz="0.18 0.0 -0.004"
  rpy="-1.57 0 0"`，几何与后轮完全一致以共用接地面），**并且**在
  `gazebo/turtlebot3_waffle_pi.gazebo.xacro` 里加 `<gazebo reference="${prefix}caster_front_link">`
  把 `mu1`/`mu2` 设成 `0.1`。**两者缺一不可** —— 漏掉 gazebo 块会用默认摩擦（≈1.0），
  前轮变成**高摩擦刹车垫**而不是万向轮（9-16 那次失败就是这个原因）。
- **验收（实测）**：pitch Δ **+11.70° → +0.30°**；TCP 水平误差 **0.196 m → 0.008 m**；
  驱动无刹车（转 69.7°/平移 0.209 m，均 ≈理论值 70%，差值是一秒的 cmd_vel 启动延迟）；
  底盘 z 仍 0.0000（与后轮同一接地面）。

### 抓取点的自检（2026-09-18 加，防"照偏掉的点下爪")

`grasp_phase.py` 在**发出抓取请求前**会对最终抓取点做一次自检（`_target_suspect`），
不通过就重拍，再不过就**拒抓**。原因是一次实跑：手指合到 57.0 mm **空合**
（TF 间隙 76→57 全程无阻挡，而罐子窄边 66 mm），抓空。

**它问的问题**：底盘在「微调收敛」和「抓取点重拍」之间**没动过**，所以这两次对同一个
静止物体的测量本该一致 —— 对不上就说明至少有一次是错的。

| 闸门 | 阈值 | 说明 |
|---|---|---|
| 置信度 | `< 0.70` | 失败那次 0.53，成功那次 0.89 |
| **横向差** | `> 0.012 m` | 两指**闭合方向**，严 —— 没有别的机制能补救它 |
| 距离差 | `> 0.060 m` | 松（**必须大于 `_nudge` 的 0.030**）—— 距离差有 `_nudge` 专门去补，自检只在 nudge 也救不回时才拦 |

> ⚠️ **必须分方向判，看总差异大小是判不出来的。** 两次实跑的实测：
> 成功那次微调 (0.377,+0.000) vs 重拍 (0.402,+0.000) → 差 **25 mm**，但**全在前向**
> （只是"多伸 2 cm"，罐子仍在两指之间 ✓）；
> 失败那次 (0.407,−0.000) vs (0.412,−0.019) → 差 19.6 mm，其中**横向 19 mm**（合空 ✗）。
> **成功那次的差异反而更大** —— 所以只能按方向判。
>
> 旧的安全网是 `abs(by) > 0.020`，那次横向偏 0.019 **差 1 mm 没触发**；另一路拿
> `standoff`（预期距离）当参照，而那次站位被外推得更远、预期值从 0.370 漂到 0.400，
> 于是距离那路也没触发。**新自检不依赖任何"预期值"。**

### 比赛答案 JSON（评分用）

`patrol_task.py` 顶部有评分相关的配置，换队伍 / 换题时务必检查：

- `GROUP_NUMBER`：组号，答案文件会命名为 `<GROUP_NUMBER>_answer.json`。
- `TARGET_CLASSES_JSON`：本轮需写入答案的目标类别（规范名），须与裁判发布的类别集合完全一致。
- `NAME_TO_JSON`：内部类别名 → 评分规范类别名的映射。
- `ANSWER_OUTPUT_DIR`：答案输出目录（默认 `~/turtlebot3_ws/submissions`，可用环境变量覆盖）。

## 文档说明

- `docs/turtlebot3-fr3-fortress-integration.md` — TB3+FR3 在 Fortress 的集成改动清单、控制器/传感器验证结果、转运姿态关节值。
- `docs/task-navigation.md` — 任务节点的初始位姿、访问顺序、串行导航流程、地图坐标系约定。

## 进程清理（重启仿真前）

分终端 `Ctrl-C` 最干净；若进程残留（仿真重启后行为诡异、话题接不上），按下面清：

```bash
# 仿真 / 桥接
pkill -9 -f 'ign gazebo'
pkill -9 -f parameter_bridge
pkill -9 -f ros_gz_bridge
pkill -9 -f robot_state_publisher
# ★★ 导航：必须直接杀 component_container —— Nav2 的节点是 **compose 在一个
#    `component_container_isolated` 进程里的**（amcl / controller_server /
#    planner_server / bt_navigator / map_server / behavior_server /
#    waypoint_follower / smoother_server 全都不是独立进程），
#    它们的名字**根本不出现在任何 cmdline 里** → 下面按名字点名的那几行
#    **一个也杀不掉** ✗（2026-09-18 实测：连开 4 轮仿真，攒了 4 个容器，
#    节点图里出现 2~4 份同名 amcl/controller_server，AMCL 行为随即错乱）
pkill -9 -f component_container
# 下面这些是"万一没 compose / 留了独立进程"时的兜底；正常情况杀不到东西
pkill -9 -f amcl
pkill -9 -f bt_navigator
pkill -9 -f controller_server
pkill -9 -f planner_server
pkill -9 -f smoother_server
pkill -9 -f behavior_server
pkill -9 -f waypoint_follower
pkill -9 -f velocity_smoother
pkill -9 -f map_server
# cmd_vel 桥接（每一轮 launch 起一个，不点名会一路攒）
pkill -9 -f cmd_vel_relay
# 抓取（move_group / MTC 抓取节点）
pkill -9 -f move_group
pkill -9 -f grasp_node
# 视觉服务：正常流程下它**不是独立进程**，寄生在 patrol_task 里（下一行一并清掉）。
# 只有 enable_vision:=true 的独立调试路径才会有这个进程。
pkill -9 -f detect_grasp_target
# 任务节点（★ 视觉服务寄生在这里，清它就等于清掉视觉服务）
pkill -9 -f patrol_task
pkill -9 -f dining_grasp_task
```

> 残留检查：`nvidia-smi --query-compute-apps=pid,used_memory --format=csv` 应该**没有**
> python 进程占显存。上面这套 `pkill` 如果漏了，patrol 进程会活着继续占 ~2.8 GiB，
> 下一次启动的模型加载就可能 CUDA OOM（2026-09-16 就吃过这个亏）。
>
> 进程都清干净之后，还建议清一次 fastrtps 的共享内存残留：
> `rm -f /dev/shm/fastrtps_*`（**确认没有 ROS 进程活着再清**）。不清的话每次启动
> 都会刷一堆 `RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7413:
> open_and_lock_file failed`（无害但淹没日志）。实测攒了 274 个文件 / 8.3 MB。
>
> 判"清干净了没有"最快的办法：`ros2 node list --no-daemon` 应该是**空的**
> （`ros2 node list` 会读 daemon 的缓存，可能显示已经死掉的节点 ✗）。

> 注意：`pkill -f` 用扩展正则，**模式里不要写 `\|`**（那会变成字面竖线，匹配不到）。
> 需要多选就用真正的 `|`，或像上面一样逐个点名。
>
> `move_group` 残留时抓取服务端口会占着，新起的 `grasp_service.launch.py`
> 看起来"起来了"但服务调不通 —— 重启仿真前务必清掉。
