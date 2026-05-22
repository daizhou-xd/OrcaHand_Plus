# My ORCA — Camera-Driven Hand Teleoperation & Grasping

Real-time camera-driven teleoperation and grasping system for the ORCA Hand.
Captures 21 hand landmarks via MediaPipe, maps them to the ORCA Hand's 17
joint angles (0–360° servo range), and drives Feetech ST3215 servos.

---

## Directory Structure

```
my_orca/
├── orca_core/                  # Core ORCA Hand control package
│   ├── base_hand.py            #   Abstract hand base: interpolation, safety clamping
│   ├── hardware_hand.py        #   OrcaHand class: connect, calibrate, deg↔motor conversion
│   ├── joint_position.py       #   OrcaJointPositions immutable joint data container
│   ├── hand_config.py          #   YAML config loading and validation
│   ├── calibration.py          #   Calibration result storage
│   ├── constants.py            #   Constants (mode mappings, joint names, etc.)
│   ├── hardware/
│   │   ├── motor_client.py     #   Abstract MotorClient interface
│   │   ├── feetech_client.py   #   Feetech STS/SMS servo communication
│   │   ├── dynamixel_client.py #   Dynamixel servo communication (fallback)
│   │   ├── mock_dynamixel_client.py  #   Mock client (no-hardware testing)
│   │   └── feetech/            #   Feetech SCServo SDK (low-level serial protocol)
│   ├── utils/utils.py          #   YAML I/O, port auto-detection, interpolation
│   ├── models/                 #   Hand model configs (v1/v2, left/right)
│   └── version.py
├── config/
│   ├── config.yaml             #   Right-hand Feetech config (COM13, 1M baud)
│   └── calibration.yaml        #   Generated calibration data (motor limits)
├── hand_tracker.py             # MediaPipe camera hand tracking module
├── hand_mapper.py              # Hand landmark → OrcaHand joint angle mapping (0–360°)
├── main_grasp.py               # Main camera teleop + grasping demo
├── slider_control.py           # Tkinter slider GUI — 设计预定动作
├── debug_joint.py              # Interactive single-joint debug / 输入角度设动作
├── manual_calibrate_v2.py      # Manual calibration (raw 0–4095 sweep)
└── requirements.txt            # Python dependencies
```

---

## Quick Start

### 1. Install Dependencies

```bash
cd my_orca
pip install -r requirements.txt
```

| Package | Purpose |
|---|---|
| `numpy` | Numerical computation |
| `pyyaml` | Config file parsing |
| `pyserial` | Serial port communication |
| `dynamixel-sdk` | Dynamixel SDK (required by orca_core, even for Feetech) |
| `mediapipe` | Hand landmark detection |
| `opencv-python` | Camera capture and visualization |

### 2. Check Hardware

- **USB-to-serial adapter** → PC, appears as `COM13` in Device Manager
- **12V power supply** → servos powered, LEDs lit
- **Servo IDs** 1–17 (factory default)

> **Tip:** If not COM13, edit the `port` field in [config/config.yaml](config/config.yaml).
> Auto-detection is also supported.

### 3. Calibrate (First Time)

```bash
python manual_calibrate_v2.py
```

Drives each joint from midpoint (2048) toward both limits in ±5 raw steps.
Press **Enter** at each mechanical limit to record, **Q** to skip a joint.
Stores raw 0–4095 limits directly in `calibration.yaml`.

To calibrate just one joint:

```bash
python manual_calibrate_v2.py --joint thumb_mcp
```

### 4. Run Camera Teleop

```bash
python main_grasp.py
```

Place your right hand in the camera view — hand pose maps to the ORCA Hand in real time.
Making a fist triggers auto-grasp. Press `ESC` to quit.

---

## All Scripts

### manual_calibrate_v2.py — Calibration

```
python manual_calibrate_v2.py [--config PATH] [--joint JOINT]
```

**唯一的标定方法。** 从 2048 中点出发，每次 ±5 raw 步进，
到达机械极限按 **Enter** 记录，按 **Q** 跳过。限位以 raw 整数 0–4095 存入 `calibration.yaml`。

### main_grasp.py — Camera Teleop & Grasping

```
python main_grasp.py [--config PATH] [--camera ID] [--mode MODE]
                     [--mock] [--no-auto-grasp] [--grasp-threshold DEG]
                     [--skip-init] [--force-calibrate]
```

| Argument | Default | Description |
|---|---|---|
| `--config` | `config/config.yaml` | Config file path |
| `--camera` | `0` | Camera device ID |
| `--mode` | `track` | `track` / `power` / `pinch` / `cycle` |
| `--mock` | off | Mock hand (no hardware) |
| `--no-auto-grasp` | off | Manual grasp trigger only |
| `--grasp-threshold` | `150` | Flexion threshold in servo degrees |
| `--skip-init` | off | Skip joint init (faster, if calibrated) |

**Modes:**

| Mode | Description |
|---|---|
| `track` | Camera teleop (default). Hand motions → ORCA Hand in real time |
| `power` | Execute power grasp, Enter to release |
| `pinch` | Execute pinch grasp, Enter to release |
| `cycle` | Cycle through OPEN → PINCH → POWER |

**Keyboard (track mode):**

| Key | Action |
|---|---|
| `ESC` | Quit |
| `SPACE` | Toggle auto-grasp |
| `G` | Power grasp |
| `P` | Pinch grasp |
| `O` | Open hand |
| `N` | Neutral position |
| `T` | Toggle torque |

### slider_control.py — 设计预定动作

```
python slider_control.py [--config PATH] [--mock] [--skip-init]
```

Tkinter 滑条面板，每个关节 0–360°。**用于设计预定动作/姿态，不是标定工具。**
拖动滑条找到想要的手势，记录下各关节角度值，填入抓取预设。

### debug_joint.py — 输入角度调试

```
python debug_joint.py                              # 交互模式
python debug_joint.py --list                       # 列出所有关节
python debug_joint.py --joint index_mcp --angle 90 # 单次设角度
python debug_joint.py --joint thumb_mcp --sweep    # 扫描极限范围
```

交互模式下直接输入 `<关节名> <角度>` 设定单个关节位置，用于**手动编制动作**。

---

## Configuration Guide

[config/config.yaml](config/config.yaml) — key sections:

### Joint Limits

```yaml
joint_limits:
  thumb_mcp: [500, 3500]   # raw 0–4095 mechanical limits
  index_mcp: [500, 3500]
  ...
```

These are the **mechanical limits** in raw servo units (0–4095).
The system maps them linearly to 0–360°:
- 0 raw → 0°, 2047 raw → 180°, 4095 raw → 360°
- Joint limits clamp the output to the safe range

### Neutral Position

```yaml
neutral_position:
  thumb_mcp: 180   # degrees (180° = servo midpoint 2047)
  index_mcp: 180
  ...
```

All joints default to 180° (servo midpoint). Adjust per joint if needed.

### Joint → Motor Map

```yaml
joint_to_motor_map:
  thumb_mcp: 15
  index_pip: -1    # negative = inverted rotation
  wrist: -17
```

| Joint | Motor | Inverted | Description |
|---|---|---|---|
| `thumb_mcp` | 15 | no | Thumb MCP (basal) |
| `thumb_abd` | 13 | no | Thumb abduction |
| `thumb_pip` | 14 | no | Thumb PIP |
| `thumb_dip` | 16 | yes | Thumb DIP |
| `index_abd` | 3 | no | Index abduction |
| `index_mcp` | 2 | no | Index MCP |
| `index_pip` | 1 | yes | Index PIP |
| `middle_abd` | 4 | no | Middle abduction |
| `middle_mcp` | 9 | no | Middle MCP |
| `middle_pip` | 8 | yes | Middle PIP |
| `ring_abd` | 12 | yes | Ring abduction |
| `ring_mcp` | 10 | no | Ring MCP |
| `ring_pip` | 11 | yes | Ring PIP |
| `pinky_abd` | 5 | no | Pinky abduction |
| `pinky_mcp` | 7 | no | Pinky MCP |
| `pinky_pip` | 6 | no | Pinky PIP |
| `wrist` | 17 | yes | Wrist flex/extend |

### Angle System

All joint positions are in **servo degrees (0–360°)**:
- **0°** = fully extended (servo raw 0)
- **180°** = neutral / midpoint (servo raw 2047)
- **360°** = fully flexed (servo raw 4095)

The mapping is **linear**: `raw = (deg / 360) × 4095`, clamped to `joint_limits`.
No gear ratios or ROM tables needed.

---

## Module API

### HandTracker

```python
from hand_tracker import HandTracker

with HandTracker(camera_id=0) as tracker:
    while True:
        detected = tracker.read_frame()
        if detected:
            lm = tracker.landmarks        # (21, 3) pixel coords, smoothed
            wl = tracker.world_landmarks  # (21, 3) world coords (meters)
        key = tracker.show("Hand Tracker")
        if key == 27:
            break
```

### HandToOrcaMapper

```python
from hand_mapper import HandToOrcaMapper

# limits_dict: joint_name → [min_raw, max_raw] from config
mapper = HandToOrcaMapper(flip_hand=False, limits_dict=hand.config.joint_limits_dict)
joint_angles = mapper.compute_joint_angles(landmarks_px, landmarks_world)
# → {"index_mcp": 180.0, "thumb_abd": 210.0, ...}  (0–360°)
```

### detect_grasp_state()

```python
from hand_mapper import detect_grasp_state

is_grasping, strength = detect_grasp_state(
    joint_angles,
    flex_threshold=150.0   # servo degrees, 150° ≈ halfway between open and fist
)
# is_grasping: bool
# strength: float ∈ [0, 1]
```

### Grasp Presets

```python
from main_grasp import GRASP_POWER, GRASP_PINCH, GRASP_OPEN

hand.set_joint_positions(GRASP_POWER, num_steps=20, step_size=0.02)
```

| Preset | Description |
|---|---|
| `GRASP_POWER` | All fingers fully flexed (~290°) |
| `GRASP_PINCH` | Thumb + index flexed, others extended |
| `GRASP_OPEN` | All fingers extended (~130–160°) |

---

## Workflow

### First-Time Setup

```bash
pip install -r requirements.txt

# Calibrate every joint (mechanical limits)
python manual_calibrate_v2.py

# Test camera tracking (no hardware)
python main_grasp.py --mock

# Run with hardware
python main_grasp.py
```

### Designing Preset Poses

```bash
# Use slider to find desired pose, note the angles
python slider_control.py

# Or command a single joint by angle
python debug_joint.py --joint index_mcp --angle 200
```

Then copy the angle values into `GRASP_POWER` / `GRASP_PINCH` / `GRASP_OPEN` in `main_grasp.py`.

### Daily Use

```bash
# Camera teleop (skip init if already calibrated)
python main_grasp.py --skip-init

# Cycle grasp presets
python main_grasp.py --mode cycle

# Single preset
python main_grasp.py --mode pinch
```

---

## Troubleshooting

### Connection Failed

1. Check USB adapter is plugged in, visible in Device Manager
2. Check 12V power is on, servo LEDs lit
3. Verify `port` in config.yaml — or let auto-detection find it

### Servos Not Moving

1. Verify 12V power is stable
2. Check servo cables are firmly connected
3. Try lowering `calibration_current` in config

### Servo Overload (Flashing Red LED)

- Check for over-tightened tendons or mechanical obstruction
- Press `T` to disable torque, manually check joint mobility

### Camera Not Detecting Hand

1. Ensure hand is in frame with adequate lighting
2. Face palm toward camera, fingers spread
3. Lower `min_detection_confidence` in [hand_tracker.py](hand_tracker.py)
