# `camera3d_hand_to_api.py` 代码详解

> 本文档对 `camera3d_hand_to_api.py` 进行逐行注解，并说明如果你的手部硬件关节映射不同，需要修改哪些地方。

---

## 一、文件概览

这个脚本实现了一条完整的 pipeline：

```
摄像头 → MediaPipe 手部关键点(21点3D) → 几何计算关节角度 → EMA平滑 → HTTP API 发送舵机指令
```

同时支持：过流保护轮询、中文 OSD 叠加显示、可选反馈读取、干运行模式（不发送指令只显示）。

---

## 二、逐段详解

### 2.1 导入与初始化（第1-24行）

```python
import os
import sys
import argparse
import math
import time
import tempfile
from dataclasses import dataclass
from typing import Dict, Optional, Set


def _ocra_temp_dir() -> str:
    # 获取系统临时目录下的 ocra 专用子目录
    return os.path.join(tempfile.gettempdir(), "ocra_py_action")


def _configure_pycache_prefix() -> None:
    # 将 Python 字节码缓存重定向到临时目录，避免在项目目录中产生 __pycache__ 污染
    try:
        base = os.environ.get("OCRA_PYCACHE_DIR") or os.path.join(_ocra_temp_dir(), "pycache")
        os.makedirs(base, exist_ok=True)
        sys.pycache_prefix = base  # type: ignore[attr-defined]
    except Exception:
        pass


_configure_pycache_prefix()  # 模块加载时立即执行
```

### 2.2 条件导入（第26-51行）

```python
import numpy as np      # 矩阵/向量运算
import requests         # HTTP API 调用

try:
    import cv2                          # OpenCV：摄像头采集 + 窗口显示
except Exception:
    cv2 = None

try:
    import yaml                         # PyYAML：读取配置文件
except Exception:
    yaml = None

try:
    import mediapipe as mp              # Google MediaPipe：手部关键点检测
except Exception:
    mp = None

try:
    from PIL import Image, ImageDraw, ImageFont  # Pillow：中文文字叠加（抗锯齿）
except Exception:
    Image = None
    ImageDraw = None
    ImageFont = None
```

**⚠️ 核心依赖：** 脚本真正依赖 `numpy + requests + mediapipe + opencv`。`yaml` 和 `PIL` 是可选增强。

### 2.3 数据结构（第54-58行）

```python
@dataclass(frozen=True)
class JointScale:
    rad_min: float    # 关节最小弧度
    rad_max: float    # 关节最大弧度
```

这个不可变数据类代表一个关节的机械运动范围。**如果你的手的某个关节转动范围不同，这就是你需要改的地方**——通过配置文件 `config_orca.yaml` 中的 `joint_scale` 段来改，而不是改代码。

### 2.4 基础几何工具函数（第60-94行）

```python
def _unit(v: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    # 向量归一化，零向量安全（返回零向量而非报错）
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.zeros_like(v)
    return v / n


def _angle_at(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """计算 ∠ABC（以 B 为顶点），单位弧度，范围 [0, π]"""
    ba = a - b
    bc = c - b
    ba_u = _unit(ba)
    bc_u = _unit(bc)
    dot = float(np.clip(np.dot(ba_u, bc_u), -1.0, 1.0))  # clip防止数值误差导致 acos 报错
    return float(math.acos(dot))


def _flexion_from_angle(angle_abc: float) -> float:
    """
    将"内角"转换为"弯曲量"。
    手指伸直时 A-B-C 几乎在一条直线上 → angle ≈ π
    弯曲 = π - angle，最小为 0（即完全伸直）
    例：完全伸直 angle≈π → flexion≈0；弯曲90° angle≈π/2 → flexion≈π/2
    """
    return max(0.0, math.pi - float(angle_abc))


def _signed_angle_about_axis(v_from: np.ndarray, v_to: np.ndarray, axis: np.ndarray) -> float:
    """
    绕指定轴的有符号角度（单位弧度），范围 [-π, π]。
    使用 atan2 实现，结果的正负由右手定则决定。
    """
    a = _unit(axis)
    v1 = _unit(v_from)
    v2 = _unit(v_to)
    cross = np.cross(v1, v2)
    sin_term = float(np.dot(a, cross))      # 轴方向上的叉积分量 → sin
    cos_term = float(np.clip(np.dot(v1, v2), -1.0, 1.0))  # 点积 → cos
    return float(math.atan2(sin_term, cos_term))
```

### 2.5 关节值映射与限幅（第97-129行）

```python
def _clamp_to_scale(joint: str, value: float, scales: Dict[str, JointScale]) -> float:
    """
    将关节角度值限制在 joint_scale 范围内。
    处理 lo>hi（舵机反转）的情况——如果 rad_max < rad_min，则反转限幅边界。
    """
    sc = scales.get(joint)
    if sc is None:
        return float(value)               # 无配置 → 不限制
    lo, hi = float(sc.rad_min), float(sc.rad_max)
    if lo <= hi:
        return float(np.clip(value, lo, hi))
    return float(np.clip(value, hi, lo))  # rad_max < rad_min 时反转


def _map_flex_to_joint_range(flex_rad: float, joint: str, scales: Dict[str, JointScale]) -> float:
    """
    将弯曲角(0 ~ π/2)线性映射到关节的机械弧度范围。
    flex_rad = 0     → 输出 = rad_min（伸直/零位）
    flex_rad = π/2   → 输出 = rad_max（完全弯曲）
    
    支持 rad_min < 0 的情况（双向关节）。
    """
    sc = scales.get(joint)
    if sc is None:
        rad_min, rad_max = 0.0, 1.2       # 默认范围，建议替换为实际值
    else:
        rad_min, rad_max = float(sc.rad_min), float(sc.rad_max)

    if rad_max == rad_min:
        return float(rad_min)

    if rad_min >= 0.0:
        # 单向关节（纯弯曲）：flex/1.57 作为 0→1 的插值因子
        t = float(np.clip(flex_rad / (math.pi / 2.0), 0.0, 1.0))
        lo, hi = (rad_min, rad_max) if rad_min <= rad_max else (rad_max, rad_min)
        return float(np.clip(t * hi, lo, hi))

    # 双向关节（如侧摆）：允许负方向
    t = float(np.clip(flex_rad / (math.pi / 2.0), -1.0, 1.0))
    return float(np.clip(t * max(abs(rad_min), abs(rad_max)), rad_min, rad_max))
```

### 2.6 EMA 平滑（第132-141行）

```python
def _ema_update(prev: Dict[str, float], cur: Dict[str, float], ema: float) -> Dict[str, float]:
    """
    指数加权移动平均（EMA）：
        new_value = α × prev + (1-α) × current
    α 越大越平滑（响应越慢），默认 0.75。
    如果某一帧出现了之前没见过的关节，直接加入。
    """
    if not prev:
        return dict(cur)                  # 第一帧 → 无历史
    out: Dict[str, float] = dict(prev)
    for k, v in cur.items():
        if k in out:
            out[k] = float(ema) * float(out[k]) + (1.0 - float(ema)) * float(v)
        else:
            out[k] = float(v)
    return out
```

### 2.7 配置文件读取（第144-174行）

```python
def _read_config(config_path: str) -> tuple[Dict[str, JointScale], Set[str]]:
    """
    读取 YAML 配置文件，返回：
    1. scales:  关节名 → JointScale(rad_min, rad_max)
    2. allowed_joints: joint_to_servo_map 中定义的所有关节名集合
    """
    if yaml is None:
        return {}, set()

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            return {}, set()

        # 提取"允许的关节名"——来自 joint_to_servo_map 的 keys
        joint_to_servo_map = data.get("joint_to_servo_map", {})
        allowed_joints: Set[str] = set()
        if isinstance(joint_to_servo_map, dict):
            allowed_joints = {str(k) for k in joint_to_servo_map.keys()}

        # 提取每个关节的弧度范围
        joint_scale = data.get("joint_scale", {})
        scales: Dict[str, JointScale] = {}
        if isinstance(joint_scale, dict):
            for joint_name, sc in joint_scale.items():
                if not isinstance(sc, dict):
                    continue
                try:
                    scales[str(joint_name)] = JointScale(
                        rad_min=float(sc.get("rad_min", 0.0)),
                        rad_max=float(sc.get("rad_max", 0.0)),
                    )
                except Exception:
                    continue
        return scales, allowed_joints
    except Exception:
        return {}, set()
```

**🔑 关键：** 配置文件中的 `joint_to_servo_map` 的两重作用：
1. 定义"哪些关节是允许发送的"（allowed_joints）
2. 供 API 服务端做"关节名 → 舵机ID"的映射

### 2.8 HTTP API 通信（第177-253行）

```python
def _post_json(base: str, path: str, json_body):
    """发送 POST 请求，超时 2.5 秒"""
    url = base.rstrip("/") + path
    r = requests.post(url, json=json_body, timeout=2.5)
    r.raise_for_status()
    return r.json()


def _get_json(base: str, path: str):
    """发送 GET 请求，超时 2.5 秒"""
    url = base.rstrip("/") + path
    r = requests.get(url, timeout=2.5)
    r.raise_for_status()
    return r.json()


def _post_config_connect(base: str, config_path: str) -> None:
    """
    启动时通知 API 服务端：
    1. POST /config → 告诉服务端用哪个配置文件
    2. POST /connect → 告诉服务端连接舵机
    """
    url_config = base.rstrip("/") + "/config"
    r = requests.post(url_config, json=config_path, timeout=5)
    r.raise_for_status()

    url_connect = base.rstrip("/") + "/connect"
    try:
        r = requests.post(url_connect, json={}, timeout=5)
        r.raise_for_status()
    except Exception:
        return  # 端口可能被占用，不影响识别功能
```

**API 端点总结：**

| 方法 | 路径 | 用途 |
|------|------|------|
| POST | `/config` | 传递配置文件路径 |
| POST | `/connect` | 连接舵机 |
| GET | `/version` | 获取连接状态 + 过流状态 |
| POST | `/joints/position` | **发送关节角度指令**（核心） |
| POST | `/overcurrent/reset` | 复位过流保护 |
| GET | `/motors/position` | 读取舵机原始位置（feedback） |
| GET | `/joints/position` | 读取关节角度（feedback） |

### 2.9 MediaPipe 关键点提取（第255-266行）

```python
def _extract_landmarks_3d(results, frame_w: int, frame_h: int) -> Optional[np.ndarray]:
    """
    从 MediaPipe 检测结果中提取 21 个手部关键点的伪3D坐标。
    返回 (21, 3) 的 numpy 数组。
    
    z 值在 MediaPipe 中是与 x 同量纲的归一化值（约 0~1 范围），
    所以同样乘以 frame_w 转换为像素尺度。
    """
    if not results.multi_hand_landmarks:
        return None
    hand = results.multi_hand_landmarks[0]  # 取第一只手
    pts = []
    for lm in hand.landmark:
        pts.append([lm.x * frame_w, lm.y * frame_h, lm.z * frame_w])
    return np.asarray(pts, dtype=np.float32)
```

**MediaPipe 21点手部关键点：**

```
        8   12   16   20
        |    |    |    |
        7   11   15   19
        |    |    |    |
        6   10   14   18
        |    |    |    |
     5--+----9----13---17
      \                 /
       2   3    4
       |   |    |
       1---0----+
    
     0: 手腕 (Wrist)        5: 食指 MCP      10: 中指 PIP      15: 无名指 DIP
     1: 拇指 CMC            6: 食指 PIP      11: 中指 DIP      16: 无名指 TIP
     2: 拇指 MCP            7: 食指 DIP      12: 中指 TIP      17: 小指 MCP
     3: 拇指 IP             8: 食指 TIP      13: 无名指 MCP    18: 小指 PIP
     4: 拇指 TIP            9: 中指 MCP      14: 无名指 PIP    19: 小指 DIP
                                                             20: 小指 TIP
```

### 2.10 显示文本列表生成（第269-291行）

```python
def _display_keys_all(allowed_joints: Set[str]) -> list[str]:
    """
    生成 OSD 显示用的关节名列表（按逻辑顺序排列）。
    如果 allowed_joints 非空，则只显示配置中已定义的关节。
    """
    ordered = [
        "拇指根部", "拇指侧摆", "拇指近端", "拇指远端",
        "食指侧摆", "食指近端", "食指远端",
        "中指侧摆", "中指近端", "中指远端",
        "无名指侧摆", "无名指近端", "无名指远端",
        "小指侧摆", "小指近端", "小指远端",
        "腕部关节",
    ]
    if allowed_joints:
        return [k for k in ordered if k in allowed_joints]
    return ordered
```

### 2.11 中文字体渲染（第294-361行）

```python
def _find_font_path(preferred: Optional[str] = None) -> Optional[str]:
    # 自动探测 Windows 系统中文字体（微软雅黑 → 黑体 → 宋体）
    if preferred:
        return preferred
    candidates = [
        r"C:\\Windows\\Fonts\\msyh.ttc",
        r"C:\\Windows\\Fonts\\msyh.ttf",
        r"C:\\Windows\\Fonts\\simhei.ttf",
        r"C:\\Windows\\Fonts\\simsun.ttc",
    ]
    for p in candidates:
        try:
            with open(p, "rb"):
                return p
        except Exception:
            continue
    return None


def _draw_text_pil(frame_bgr, lines, font_path, font_size):
    """
    用 Pillow 渲染中文文字叠加到 BGR 图像上。
    支持 (文本, BGR颜色元组) 格式的彩色文字。
    """
    img = Image.fromarray(frame_bgr[:, :, ::-1])  # BGR → RGB
    draw = ImageDraw.Draw(img)
    fp = _find_font_path(font_path)
    try:
        font = ImageFont.truetype(fp, font_size) if fp else ImageFont.load_default()
    except Exception:
        font = ImageFont.load_default()

    x, y = 10, 10
    line_h = int(font_size * 1.25)
    for entry in lines:
        if isinstance(entry, tuple) and len(entry) == 2:
            line, bgr = entry
            color = (int(bgr[2]), int(bgr[1]), int(bgr[0]))  # BGR → RGB for PIL
            draw.text((x, y), str(line), font=font, fill=color)
        else:
            draw.text((x, y), str(entry), font=font, fill=(255, 255, 255))
        y += line_h
    return np.asarray(img)[:, :, ::-1]  # RGB → BGR


def _draw_text_opencv(frame_bgr, lines):
    """
    用 OpenCV putText 渲染文字（英文正常，中文会乱码）。
    """
    if cv2 is None:
        return frame_bgr
    x, y = 10, 30
    for entry in lines:
        if isinstance(entry, tuple) and len(entry) == 2:
            line, bgr = entry
            color = bgr
        else:
            line = entry
            color = (255, 255, 255)
        cv2.putText(frame_bgr, str(line), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
        y += 22
    return frame_bgr
```

### 2.12 🔥 核心：3D 几何 → 关节角度计算（第364-506行）

这是整个文件最重要的部分。

#### 2.12.1 手掌坐标系建立（第371-376行）

```python
def _compute_joint_commands_3d(pts, scales, allowed_joints, enable_splay, splay_invert):
    # ------ 建立手掌坐标系 (pseudo-3D) ------
    wrist = pts[0]                               # MediaPipe 索引0 = 手腕
    palm_x = _unit(pts[5] - pts[17])             # 小指MCP → 食指MCP ≈ 手掌左右轴
    palm_y = _unit(pts[9] - wrist)               # 手腕 → 中指MCP ≈ 手掌前后轴
    palm_z = _unit(np.cross(palm_x, palm_y))     # 手掌法向量（前向/后向）
    palm_y = _unit(np.cross(palm_z, palm_x))     # Gram-Schmidt 正交化
```

**建立的三个正交轴：**
- `palm_x`：沿手掌左右方向（小指→食指）
- `palm_y`：沿手掌上下方向（手腕→中指根）
- `palm_z`：手掌法向量（垂直掌心平面）

#### 2.12.2 手指弯曲辅助函数（第378-385行）

```python
    def flex_mcp(mcp: int, pip: int) -> float:
        # MCP 弯曲：以 wrist-mcp-pip 三点计算
        return _flexion_from_angle(_angle_at(wrist, pts[mcp], pts[pip]))

    def flex_pip(mcp: int, pip: int, dip: int) -> float:
        # PIP 弯曲：以 mcp-pip-dip 三点计算
        return _flexion_from_angle(_angle_at(pts[mcp], pts[pip], pts[dip]))

    def flex_dip(pip: int, dip: int, tip: int) -> float:
        # DIP 弯曲：以 pip-dip-tip 三点计算
        return _flexion_from_angle(_angle_at(pts[pip], pts[dip], pts[tip]))
```

#### 2.12.3 四指弯曲 → "近端/远端" 二关节映射（第389-413行）

```python
    # 权重：将 MCP/PIP/DIP 的三个弯曲量融合为两个舵机指令（近端、远端）
    # ⚠️ 这是需要根据你的手调整的参数！
    prox_w_mcp, prox_w_pip, prox_w_dip = 0.45, 0.45, 0.10  # 近端舵机主要反映MCP+PIP
    dist_w_mcp, dist_w_pip, dist_w_dip = 0.10, 0.45, 0.45  # 远端舵机主要反映PIP+DIP

    fingers = {
        "食指": (5, 6, 7, 8),     # (MCP, PIP, DIP, TIP) 的 MediaPipe 索引
        "中指": (9, 10, 11, 12),
        "无名指": (13, 14, 15, 16),
        "小指": (17, 18, 19, 20),
    }
    for name, (mcp_i, pip_i, dip_i, tip_i) in fingers.items():
        f_mcp = flex_mcp(mcp_i, pip_i)    # 从手腕-MCP-PIP 计算MCP弯曲
        f_pip = flex_pip(mcp_i, pip_i, dip_i)  # 从MCP-PIP-DIP 计算PIP弯曲
        f_dip = flex_dip(pip_i, dip_i, tip_i)  # 从PIP-DIP-TIP 计算DIP弯曲

        # 加权融合：3个真实关节 → 2个舵机
        f_prox = prox_w_mcp * f_mcp + prox_w_pip * f_pip + prox_w_dip * f_dip
        f_dist = dist_w_mcp * f_mcp + dist_w_pip * f_pip + dist_w_dip * f_dip

        j_prox = f"{name}近端"    # 如 "食指近端"
        j_dist = f"{name}远端"    # 如 "食指远端"

        if (not allowed_joints) or (j_prox in allowed_joints):
            out[j_prox] = _clamp_to_scale(j_prox, _map_flex_to_joint_range(f_prox, j_prox, scales), scales)
        if (not allowed_joints) or (j_dist in allowed_joints):
            out[j_dist] = _clamp_to_scale(j_dist, _map_flex_to_joint_range(f_dist, j_dist, scales), scales)
```

**📐 为什么需要加权融合？**

ORCA Hand 每个手指只有 2 个舵机（近端+远端），但人类手指有 3 个关节（MCP/PIP/DIP）。权重决定了"3→2"的分配策略。

#### 2.12.4 拇指计算（第415-448行）

```python
    # ⚠️ 拇指根部旋转轴倾斜角 —— 这是需要根据你的机械结构调整的参数！
    tilt = math.radians(30.0)
    # 拇指CMC旋转轴：palm_z 向左倾斜30°（结合了手掌法向量和-x方向）
    root_axis = _unit(math.cos(tilt) * palm_z + math.sin(tilt) * (-palm_x))

    # 拇指根部（CMC）：计算拇指掌骨相对于手掌参考方向的旋转角
    thumb_meta = pts[2] - pts[1]                    # 拇指CMC→MCP 向量
    ref = palm_y                                     # 参考方向 = 掌面上下
    ref_p = ref - root_axis * float(np.dot(ref, root_axis))      # 去掉轴向分量
    meta_p = thumb_meta - root_axis * float(np.dot(thumb_meta, root_axis))
    thumb_root_flex = abs(_signed_angle_about_axis(ref_p, meta_p, root_axis))

    # 拇指 MCP 弯曲：1→2→3
    thumb_mcp_flex = _flexion_from_angle(_angle_at(pts[1], pts[2], pts[3]))
    # 拇指 IP 弯曲：2→3→4
    thumb_ip_flex = _flexion_from_angle(_angle_at(pts[2], pts[3], pts[4]))

    if (not allowed_joints) or ("拇指根部" in allowed_joints):
        out["拇指根部"] = _clamp_to_scale("拇指根部",
            _map_flex_to_joint_range(thumb_root_flex, "拇指根部", scales), scales)
    if (not allowed_joints) or ("拇指近端" in allowed_joints):
        out["拇指近端"] = _clamp_to_scale("拇指近端",
            _map_flex_to_joint_range(thumb_mcp_flex, "拇指近端", scales), scales)
    if (not allowed_joints) or ("拇指远端" in allowed_joints):
        out["拇指远端"] = _clamp_to_scale("拇指远端",
            _map_flex_to_joint_range(thumb_ip_flex, "拇指远端", scales), scales)
```

**🤏 拇指的特殊性：** 拇指的关节结构与四指不同（少一个关节，有对掌运动），所以需要独立处理。

#### 2.12.5 拇指侧摆（第439-448行）

```python
    # 拇指侧摆（外展/内收）：拇指在手掌平面内的左右摆角
    thumb_splay = 0.0
    if "all" in enable_splay or "thumb" in enable_splay:
        v_thumb_prox = pts[3] - pts[2]           # 拇指 MCP→IP 方向
        v_thumb_prox_palm = v_thumb_prox - palm_z * float(np.dot(v_thumb_prox, palm_z))
        ref_palm = palm_y - palm_z * float(np.dot(palm_y, palm_z))  # 参考=掌面"上"方向
        thumb_splay = _signed_angle_about_axis(ref_palm, v_thumb_prox_palm, palm_z)
        if "all" in splay_invert or "thumb" in splay_invert:
            thumb_splay = -float(thumb_splay)
    if (not allowed_joints) or ("拇指侧摆" in allowed_joints):
        out["拇指侧摆"] = _clamp_to_scale("拇指侧摆", float(thumb_splay), scales)
```

#### 2.12.6 四指侧摆（第450-489行）

```python
    # 将向量投影到手掌平面（去除法向量分量）
    def proj_palm(v: np.ndarray) -> np.ndarray:
        return v - palm_z * float(np.dot(v, palm_z))

    # 获取每根手指的 MCP→PIP 方向在手掌平面内的投影
    v_mid = proj_palm(pts[10] - pts[9])    # 中指
    v_idx = proj_palm(pts[6] - pts[5])     # 食指
    v_rng = proj_palm(pts[14] - pts[13])   # 无名指
    v_pky = proj_palm(pts[18] - pts[17])   # 小指

    # 食指侧摆：以中指方向为参考，食指偏离的角度
    if "all" in enable_splay or "index" in enable_splay:
        splay_index = _signed_angle_about_axis(v_mid, v_idx, palm_z)
        # ...

    # 中指侧摆：以手掌"上"方向为参考（因为中指是中间手指，没有天然参照）
    if "all" in enable_splay or "middle" in enable_splay:
        splay_middle = _signed_angle_about_axis(palm_y, v_mid, palm_z)
        # ...

    # 无名指、小指逻辑类似...
```

**侧摆逻辑：**
- 食指、无名指、小指：以**中指方向**为参照，计算偏角
- 中指：以**手掌中轴（palm_y）**为参照，计算偏角
- 所有侧摆绕 `palm_z`（手掌法向量）轴旋转

#### 2.12.7 腕部关节（第491-505行）

```python
    if (not allowed_joints) or ("腕部关节" in allowed_joints):
        # 腕部弯曲/伸展：手掌法向量(palm_z)相对于相机Z轴的旋转
        camera_z = np.array([0.0, 0.0, 1.0], dtype=float)
        axis = palm_x                    # 绕手掌左右轴旋转
        ref = camera_z - axis * float(np.dot(camera_z, axis))
        val = palm_z - axis * float(np.dot(palm_z, axis))
        if float(np.linalg.norm(ref)) < 1e-8 or float(np.linalg.norm(val)) < 1e-8:
            wrist_rad = 0.0
        else:
            wrist_rad = float(_signed_angle_about_axis(ref, val, axis))
        out["腕部关节"] = _clamp_to_scale("腕部关节", wrist_rad, scales)

    return out
```

**⚠️ 注意：** MediaPipe 只检测手部，没有前臂信息，所以腕部角度只是一个**近似估计**——通过手掌相对于相机的朝向推算。不是一个精确的腕部角度。

### 2.13 main() 主循环（第509-791行）

#### 2.13.1 参数解析（第510-569行）

```python
parser = argparse.ArgumentParser(...)
parser.add_argument("--base", default="http://127.0.0.1:8001")   # API 地址
parser.add_argument("--config", default="py_action/config_orca.yaml")  # 配置文件路径
parser.add_argument("--camera", type=int, default=0)             # 摄像头索引
parser.add_argument("--hz", type=float, default=10.0)            # 指令发送频率
parser.add_argument("--ema", type=float, default=0.75)           # 平滑系数
parser.add_argument("--hand", choices=["auto","left","right"], default="auto")
parser.add_argument("--dry-run", action="store_true")            # 干运行(不发送)
parser.add_argument("--no-window", action="store_true")          # 无窗口模式
parser.add_argument("--enable-splay", nargs="*", default=["all"]) # 启用侧摆的手指
parser.add_argument("--splay-invert", nargs="*", default=[])     # 反转侧摆方向的手指
parser.add_argument("--feedback", choices=["none","motors","joints"], default="none")
parser.add_argument("--auto-reconnect", default=True)            # 自动重连
```

#### 2.13.2 初始化（第571-613行）

```python
    if mp is None or cv2 is None:
        raise RuntimeError("缺少依赖：需要 mediapipe + opencv")

    # 解析侧摆设置
    enable_splay = {str(x).lower() for x in (args.enable_splay or [])}
    splay_invert = {str(x).lower() for x in (args.splay_invert or [])}

    # 读取配置
    scales, allowed_joints = _read_config(args.config)

    # 非干运行模式：通知 API 加载配置并连接
    if not args.dry_run:
        _post_config_connect(args.base, args.config)

    # 打开摄像头
    cap = cv2.VideoCapture(int(args.camera))

    # 初始化 MediaPipe Hands
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,              # 只检测一只手
        model_complexity=1,           # 中等模型精度
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    period = 1.0 / max(1e-6, float(args.hz))   # 发送周期
    prev: Dict[str, float] = {}                 # EMA 历史值
```

#### 2.13.3 主循环（第615-787行）

```python
    while True:
        # ① 读取摄像头帧
        ok, frame = cap.read()
        if not ok:
            continue

        # ② 转为 RGB 并运行 MediaPipe
        frame_h, frame_w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        # ③ 提取 3D 关键点
        pts = _extract_landmarks_3d(results, frame_w=frame_w, frame_h=frame_h)

        # ④ 左右手过滤（如果指定了 --hand left/right）
        if pts is not None:
            if results.multi_handedness:
                handed_label = results.multi_handedness[0].classification[0].label
            if args.hand != "auto" and handed_label is not None:
                want = "Left" if args.hand == "left" else "Right"
                if handed_label != want:
                    pts = None  # 忽略不符合的手

        # ⑤ 过流状态轮询（节流 0.5s）
        if (not args.dry_run) and (time.monotonic() - oc_last >= oc_poll_period):
            oc_last = time.monotonic()
            api_connected, oc_enabled, oc_default_limit, oc_tripped, _ = \
                _get_overcurrent_from_version(args.base)
            # 自动重连
            if args.auto_reconnect and not api_connected and ...:
                try:
                    _post_json(args.base, "/connect", {})
                except Exception as e:
                    last_api_error = str(e)

        # ⑥ 计算关节角度 + 发送
        if pts is not None:
            # 左手镜像（保持和2D脚本一致）
            if handed_label == "Left":
                pts = pts.copy()
                pts[:, 0] = float(frame_w) - pts[:, 0]  # 水平翻转

            cur = _compute_joint_commands_3d(pts, scales, allowed_joints,
                                              enable_splay=enable_splay,
                                              splay_invert=splay_invert)
            prev = _ema_update(prev, cur, float(args.ema))  # EMA 平滑

            # 按频率发送
            now = time.monotonic()
            if now - last_send >= period:
                last_send = now
                if not args.dry_run:
                    try:
                        if oc_tripped:
                            pass           # 过流触发 → 暂停发送
                        elif not api_connected:
                            pass           # 未连接 → 暂停发送
                        else:
                            _post_json(args.base, "/joints/position",
                                       {"positions": prev})  # 发送！
                    except Exception as e:
                        last_api_error = str(e)

            # ⑦ 准备 OSD 叠加文字（关节角度、过流状态）
            for k in show_keys:
                v = prev.get(k)
                if v is not None:
                    overlay_lines.append((f"{k}: {float(v):+.3f}", (255, 255, 255)))

        else:
            overlay_lines.append(("[3D] No hand detected", (255, 255, 255)))

        # ⑧ 渲染窗口
        if not args.no_window:
            # 画 MediaPipe 骨架
            mp_draw.draw_landmarks(frame, ...)
            # 画文字叠加（自动选择 PIL 或 OpenCV）
            frame = _draw_text_pil(...) or _draw_text_opencv(...)
            cv2.imshow("camera3d_hand_to_api", frame)

            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):   # ESC/Q 退出
                break
            if key in (ord("r"), ord("R")):  # R 复位过流
                _post_overcurrent_reset(args.base, None)
```

---

## 三、🔧 关节映射说明

### 3.1 当前项目的映射关系

这个文件使用**17个中文关节名**作为 API 通信的标识符：

| 中文关节名 | 英文含义 | 对应手指 | 运动类型 | 当前舵机ID |
|-----------|---------|---------|---------|-----------|
| 拇指根部 | Thumb CMC | 拇指 | 对掌旋转 | 15 |
| 拇指侧摆 | Thumb ABD | 拇指 | 外展/内收 | 13 |
| 拇指近端 | Thumb MCP | 拇指 | 弯曲 | 14 |
| 拇指远端 | Thumb IP | 拇指 | 弯曲 | 16 |
| 食指侧摆 | Index ABD | 食指 | 外展/内收 | 3 |
| 食指近端 | Index MCP+PIP | 食指 | 弯曲 | 2 |
| 食指远端 | Index PIP+DIP | 食指 | 弯曲 | 1 |
| 中指侧摆 | Middle ABD | 中指 | 外展/内收 | 4 |
| 中指近端 | Middle MCP+PIP | 中指 | 弯曲 | 9 |
| 中指远端 | Middle PIP+DIP | 中指 | 弯曲 | 8 |
| 无名指侧摆 | Ring ABD | 无名指 | 外展/内收 | 12 |
| 无名指近端 | Ring MCP+PIP | 无名指 | 弯曲 | 10 |
| 无名指远端 | Ring PIP+DIP | 无名指 | 弯曲 | 11 |
| 小指侧摆 | Pinky ABD | 小指 | 外展/内收 | 5 |
| 小指近端 | Pinky MCP+PIP | 小指 | 弯曲 | 7 |
| 小指远端 | Pinky PIP+DIP | 小指 | 弯曲 | 6 |
| 腕部关节 | Wrist | 手腕 | 弯曲/伸展 | 17 |

### 3.2 数据流向

```
摄像头 → MediaPipe 21点3D坐标
  → _compute_joint_commands_3d() 计算 17 个中文关节的弧度值
  → EMA 平滑
  → POST /joints/position {"positions": {"食指近端": 0.23, "拇指根部": 0.15, ...}}
  → API 服务端根据 config_orca.yaml 的 joint_to_servo_map
    将 中文关节名 映射到 舵机ID，发送位置指令
```

---

## 四、🔄 如果你的关节映射不同，需要修改的地方

### 4.1 关卡1：只需改配置文件（推荐）

如果你的**舵机 ID 分配**不同，但关节的**中文名称保持不变**，只需修改 `config_orca.yaml`：

```yaml
# config_orca.yaml
joint_to_servo_map:
  食指远端: 1      # ← 改成你的舵机ID
  食指近端: 2      # ← 改成你的舵机ID
  食指侧摆: 3      # ← ...
  # ... 依此类推

joint_scale:
  拇指近端:
    rad_min: 0.0   # ← 改成你测出来的最小弧度
    rad_max: 1.0   # ← 改成你测出来的最大弧度
  # ... 依此类推
```

**你需要确定的数值：**
- 每个关节对应哪个舵机 ID（1~N）
- 每个关节的 `rad_min` 和 `rad_max`（机械限位弧度值）

运行命令时通过 `--config` 指定你的配置文件：
```bash
python camera3d_hand_to_api.py --config my_config.yaml
```

### 4.2 关卡2：关节名不同 → 改代码中的字符串

如果你的 API 期望**不同的关节名称**（比如英文名 `thumb_mcp` 而非 `拇指近端`），需要修改以下函数中的字符串：

**`_compute_joint_commands_3d()` 函数（第364-506行）：**

```python
# 修改前（中文名）：
out["拇指根部"] = ...
out["拇指侧摆"] = ...
out["拇指近端"] = ...
out["拇指远端"] = ...
out["食指近端"] = ...
out["食指远端"] = ...
out["食指侧摆"] = ...
# ... 等等

# 修改后（英文名示例）：
out["thumb_mcp"] = ...
out["thumb_abd"] = ...
out["thumb_pip"] = ...
out["thumb_dip"] = ...
out["index_mcp"] = ...
out["index_pip"] = ...
out["index_abd"] = ...
# ... 等等
```

**`_display_keys_all()` 函数（第269-291行）：** 同步修改显示列表：
```python
ordered = [
    "thumb_mcp", "thumb_abd", "thumb_pip", "thumb_dip",
    "index_abd", "index_mcp", "index_pip",
    # ...
]
```

### 4.3 关卡3：手指数目/关节数目不同 → 改计算逻辑

如果你的手**不是标准的 5指×3关节 结构**，需要修改：

#### A. 四指弯曲计算（第389-413行）

```python
# 当前代码假设：5根手指，每根 4 个关键点（MCP/PIP/DIP/TIP）
fingers = {
    "食指": (5, 6, 7, 8),
    "中指": (9, 10, 11, 12),
    "无名指": (13, 14, 15, 16),
    "小指": (17, 18, 19, 20),
}

# 如果你的手只有3根手指，删掉不需要的项即可
# 如果你的手的舵机不区分"近端/远端"，需要修改融合逻辑
```

#### B. MCP+PIP+DIP → 近端+远端 的权重（第390-391行）

```python
# 当前权重：
prox_w_mcp, prox_w_pip, prox_w_dip = 0.45, 0.45, 0.10
dist_w_mcp, dist_w_pip, dist_w_dip = 0.10, 0.45, 0.45

# 如果你是 1个舵机控制整根手指 → 改为一组权重
# 如果你是 3个舵机分别控制 MCP/PIP/DIP → 不需要融合，直接输出
```

#### C. 拇指根部倾斜角（第420行）

```python
tilt = math.radians(30.0)   # ← 这是拇指CMC旋转轴相对于手掌法向量的倾斜角
```

**这个值取决于你拇指舵机的安装方向。** 如果发现拇指根部运动方向不对（比如弯曲识别成了侧摆），调整这个角度。

### 4.4 关卡4：API 协议不同 → 改发送逻辑

如果 API 的请求格式不是 `{"positions": {关节名: 弧度值}}`：

```python
# 当前发送格式（第682行）：
_post_json(args.base, "/joints/position", {"positions": prev})

# 如果 API 期望的格式不同，比如：
# {"angles": [0.1, 0.2, ...]}  → 需要转换 dict → list
# {"joint_angles": {"joint1": {"rad": 0.1}}} → 需要改变嵌套结构
```

### 4.5 关卡5：侧摆关节不存在 → 禁用

如果你的手没有某些侧摆关节，用命令行参数禁用：
```bash
# 只开启拇指侧摆，其他关闭
python camera3d_hand_to_api.py --enable-splay thumb

# 全部关闭
python camera3d_hand_to_api.py --enable-splay
```

或者直接不调用侧摆计算——在配置文件的 `joint_to_servo_map` 中不包含侧摆关节名即可。

---

## 五、📋 你需要确定的数值清单

在修改之前，请用以下清单收集你的硬件的具体数值：

| 项目 | 说明 | 如何确定 |
|------|------|---------|
| **舵机 ID 映射** | 每个关节对应哪个舵机 ID | 查看硬件接线/测试每个舵机 |
| **rad_min / rad_max** | 每个关节的机械弧度范围 | 手动控制舵机到极限位置，记录弧度值 |
| **舵机方向** | 正转是弯曲还是伸展 | 如果 rad_max < rad_min，代码会自动处理反转 |
| **拇指 tilt 角** | 拇指CMC旋转轴倾斜角 | 观察拇指运动方向，调整至与实际一致 |
| **近端/远端权重** | MCP/PIP/DIP → prox/dist 的分配比例 | 调整至动作跟手自然 |
| **侧摆方向** | 外展是正角度还是负角度 | 如果不确定，用 `--splay-invert` 逐个尝试 |

---

## 六、🛠️ 调试建议

1. **先用 `--dry-run` 模式测试**：不发送指令，只在窗口查看计算出的关节值
   ```bash
   python camera3d_hand_to_api.py --dry-run --hand right
   ```

2. **关闭侧摆先调试弯曲**：
   ```bash
   python camera3d_hand_to_api.py --dry-run --enable-splay
   ```

3. **逐手指调试侧摆方向**：如果发现某根手指侧摆方向反了
   ```bash
   python camera3d_hand_to_api.py --splay-invert index  # 反转食指侧摆
   ```

4. **调整 EMA 平滑**：如果手指抖动太大，增大 ema 值（更平滑但响应慢）
   ```bash
   python camera3d_hand_to_api.py --ema 0.9
   ```

5. **调整发送频率**：如果舵机跟不上，降低频率
   ```bash
   python camera3d_hand_to_api.py --hz 5
   ```
