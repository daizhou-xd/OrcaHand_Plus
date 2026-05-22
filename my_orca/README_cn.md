# My ORCA — 摄像头手势遥操作与抓取系统

基于摄像头手势追踪的 ORCA Hand 实时遥操作抓取系统。通过 MediaPipe 捕捉人手
21 个关键点，映射到 ORCA Hand 的 17 个关节角度（0–360° 舵机范围），驱动
Feetech ST3215 舵机完成物体抓取。

---

## 目录结构

```
my_orca/
├── orca_core/                  # ORCA Hand 核心控制包
│   ├── base_hand.py            #   抽象手部基类：插值运动、安全限幅
│   ├── hardware_hand.py        #   OrcaHand 类：连接、标定、角度↔电机转换
│   ├── joint_position.py       #   OrcaJointPositions 不可变关节数据容器
│   ├── hand_config.py          #   YAML 配置文件加载与校验
│   ├── calibration.py          #   标定结果存储
│   ├── constants.py            #   常量定义
│   ├── hardware/               #   舵机驱动层
│   ├── utils/utils.py          #   YAML 读写、端口自动检测
│   └── models/                 #   手部模型配置
├── config/
│   ├── config.yaml             #   右手 Feetech 配置（COM13, 1M 波特率）
│   └── calibration.yaml        #   标定数据（电机限位）
├── hand_tracker.py             # MediaPipe 摄像头手势追踪
├── hand_mapper.py              # 人手关键点 → 关节角度映射（0–360°）
├── main_grasp.py               # 主抓取演示（摄像头遥操作）
├── slider_control.py           # Tkinter 滑条面板 — 设计预定动作
├── debug_joint.py              # 单关节调试 / 输入角度设动作
├── manual_calibrate_v2.py      # 手动标定（raw 0–4095 扫描，唯一标定方法）
└── requirements.txt            # Python 依赖
```

---

## 快速开始

### 1. 安装依赖

```bash
cd my_orca
pip install -r requirements.txt
```

| 包 | 用途 |
|---|---|
| `numpy` | 数值计算 |
| `pyyaml` | 配置文件解析 |
| `pyserial` | 串口通信 |
| `dynamixel-sdk` | Dynamixel SDK（orca_core 依赖） |
| `mediapipe` | 人手关键点检测 |
| `opencv-python` | 摄像头采集与可视化 |

### 2. 检查硬件

- **USB 转串口模块** → 电脑，设备管理器可见 `COM13`
- **12V 电源** → 舵机通电，指示灯亮
- **舵机 ID** 1–17（出厂默认）

> **提示：** 不是 COM13 就改 [config/config.yaml](config/config.yaml) 里的 `port`。也支持自动检测。

### 3. 标定（首次）

```bash
python manual_calibrate_v2.py
```

每次 ±5 raw 步进，到达机械极限按 **Enter** 记录，按 **Q** 跳过。
限位以 raw 整数 0–4095 直接存入 `calibration.yaml`。

只标定一个关节：

```bash
python manual_calibrate_v2.py --joint thumb_mcp
```

### 4. 启动摄像头遥操作

```bash
python main_grasp.py
```

摄像头窗口打开后，右手放入视野 — 手部姿势实时映射到 ORCA Hand。
握拳自动触发抓取。按 `ESC` 退出。

---

## 所有脚本速查

### manual_calibrate_v2.py — 标定（唯一方法）

```bash
python manual_calibrate_v2.py                    # 标定全部关节
python manual_calibrate_v2.py --joint thumb_mcp  # 只标定一个关节
```

从 2048 中点出发，每次 ±5 raw 步进。到达机械极限按 **Enter** 记录，按 **Q** 跳过。
限位以 raw 整数 0–4095 存入 `calibration.yaml`。

### main_grasp.py — 摄像头遥操作与抓取

```bash
python main_grasp.py --mode track        # 摄像头遥操作（默认）
python main_grasp.py --mode cycle        # 循环切换抓取姿态
python main_grasp.py --mode power        # 强力抓取
python main_grasp.py --mode pinch        # 指尖捏取
python main_grasp.py --mock              # 模拟模式（无硬件）
python main_grasp.py --skip-init         # 跳过初始化（已标定时更快）
python main_grasp.py --no-auto-grasp     # 手动按键抓取
python main_grasp.py --grasp-threshold 120  # 调低抓取触发阈值
```

**键盘控制（track 模式）：**

| 按键 | 功能 |
|---|---|
| `ESC` | 退出 |
| `SPACE` | 切换自动抓取 |
| `G` | 强力抓取 |
| `P` | 指尖捏取 |
| `O` | 张开手 |
| `N` | 回到中立位 |
| `T` | 力矩复位 |

### slider_control.py — 设计预定动作

```bash
python slider_control.py                 # 启动GUI面板
python slider_control.py --mock          # 模拟模式
```

每个关节一个滑条（0–360°）。**用于设计预定动作/姿态，不是标定工具。**
拖动滑条找到想要的手势，记录角度值，填入 `main_grasp.py` 的抓取预设。

### debug_joint.py — 输入角度设动作

```bash
python debug_joint.py                              # 交互模式
python debug_joint.py --list                       # 列出所有关节
python debug_joint.py --joint index_mcp --angle 200 # 设定指定角度
python debug_joint.py --joint thumb_mcp --sweep    # 扫描极限范围
```

交互模式下直接输入 `<关节名> <角度>` 设定单个关节，用于**手动编制动作**。

---

## 配置说明

### 角度系统

所有关节位置统一使用**舵机角度 0–360°**：

| 角度 | 含义 | raw 值 |
|---|---|---|
| 0° | 完全伸展 | 0 |
| 180° | 中位 / 默认 | 2047 |
| 360° | 完全屈曲 | 4095 |

转换关系为**线性**：`raw = (deg / 360) × 4095`，输出时夹紧到 `joint_limits` 范围。
不需要传动比、不需要 ROM 表。

### 关节限位

```yaml
joint_limits:
  thumb_mcp: [500, 3500]   # raw 0–4095 机械限位
  index_mcp: [500, 3500]
  ...
```

这些是每个关节的**机械限位**（raw 值）。标定后填入实测值。
系统自动将角度命令夹紧到此范围。

### 中立位

```yaml
neutral_position:
  thumb_mcp: 180   # 度（180° = 舵机中点 2047）
  index_mcp: 180
  ...
```

所有关节默认 180°。可根据实际需要微调。

### 关节映射表

| 关节 | 电机 | 反转 | 说明 |
|---|---|---|---|
| `thumb_mcp` | 15 | 否 | 拇指掌指关节 |
| `thumb_abd` | 13 | 否 | 拇指外展 |
| `thumb_pip` | 14 | 否 | 拇指近端指间 |
| `thumb_dip` | 16 | 是 | 拇指远端指间 |
| `index_abd` | 3 | 否 | 食指外展 |
| `index_mcp` | 2 | 否 | 食指掌指 |
| `index_pip` | 1 | 是 | 食指近端指间 |
| `middle_abd` | 4 | 否 | 中指外展 |
| `middle_mcp` | 9 | 否 | 中指掌指 |
| `middle_pip` | 8 | 是 | 中指近端指间 |
| `ring_abd` | 12 | 是 | 无名指外展 |
| `ring_mcp` | 10 | 否 | 无名指掌指 |
| `ring_pip` | 11 | 是 | 无名指近端指间 |
| `pinky_abd` | 5 | 否 | 小指外展 |
| `pinky_mcp` | 7 | 否 | 小指掌指 |
| `pinky_pip` | 6 | 否 | 小指近端指间 |
| `wrist` | 17 | 是 | 手腕屈伸 |

---

## API 参考

### HandTracker

```python
from hand_tracker import HandTracker

with HandTracker(camera_id=0) as tracker:
    while True:
        detected = tracker.read_frame()
        if detected:
            lm = tracker.landmarks        # (21, 3) 像素坐标，已平滑
            wl = tracker.world_landmarks  # (21, 3) 世界坐标（米）
        key = tracker.show("Hand Tracker")
        if key == 27:
            break
```

### HandToOrcaMapper

```python
from hand_mapper import HandToOrcaMapper

# limits_dict: joint_name → [min_raw, max_raw]
mapper = HandToOrcaMapper(flip_hand=False, limits_dict=hand.config.joint_limits_dict)
joint_angles = mapper.compute_joint_angles(landmarks_px, landmarks_world)
# → {"index_mcp": 180.0, "thumb_abd": 210.0, ...}  (0–360°)
```

### detect_grasp_state()

```python
from hand_mapper import detect_grasp_state

is_grasping, strength = detect_grasp_state(
    joint_angles,
    flex_threshold=150.0   # 舵机角度，150° ≈ 半握
)
```

### 预设抓取姿态

```python
from main_grasp import GRASP_POWER, GRASP_PINCH, GRASP_OPEN

hand.set_joint_positions(GRASP_POWER, num_steps=20, step_size=0.02)
```

| 姿态 | 说明 |
|---|---|
| `GRASP_POWER` | 五指全屈（~290°） |
| `GRASP_PINCH` | 拇指食指捏，其余伸展 |
| `GRASP_OPEN` | 五指全伸（~130–160°） |

---

## 使用流程

### 首次使用

```bash
pip install -r requirements.txt

# 标定每个关节的机械限位
python manual_calibrate_v2.py

# 模拟模式验证摄像头追踪
python main_grasp.py --mock

# 连接真实硬件
python main_grasp.py
```

### 设计预定动作

```bash
# 滑条找到想要的姿态，记下角度值
python slider_control.py

# 或者命令行指定单个关节角度
python debug_joint.py --joint index_mcp --angle 200
```

然后把角度值填入 `main_grasp.py` 里的 `GRASP_POWER` / `GRASP_PINCH` / `GRASP_OPEN`。

### 日常使用

```bash
# 摄像头遥操作（跳过初始化，快速启动）
python main_grasp.py --skip-init

# 循环切换抓取姿态
python main_grasp.py --mode cycle

# 单个姿态
python main_grasp.py --mode pinch
```

---

## 故障排除

### 连接失败

1. 确认 USB 模块已插入，设备管理器可见 COM 口
2. 确认 12V 电源已接通，舵机灯亮
3. 检查 config.yaml 的 `port` 字段，或让自动检测找

### 标定时舵机不动

1. 确认 12V 供电正常
2. 检查舵机线缆是否插紧
3. 降低 `calibration_current`（默认 120mA）

### 舵机过载（红灯闪烁）

- 检查腱绳是否过紧或关节卡阻
- 按 `T` 键关闭力矩，手动检查

### 摄像头检测不到手

1. 手在视野内，光线充足
2. 手掌朝向摄像头，五指分开
3. 降低 [hand_tracker.py](hand_tracker.py) 里的 `min_detection_confidence`
