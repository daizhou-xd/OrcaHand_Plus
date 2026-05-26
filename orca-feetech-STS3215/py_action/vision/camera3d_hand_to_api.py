import os
import sys
import argparse
import math
import time
import tempfile
from dataclasses import dataclass
from typing import Dict, Optional, Set


def _ocra_temp_dir() -> str:
    return os.path.join(tempfile.gettempdir(), "ocra_py_action")


def _configure_pycache_prefix() -> None:
    try:
        base = os.environ.get("OCRA_PYCACHE_DIR") or os.path.join(_ocra_temp_dir(), "pycache")
        os.makedirs(base, exist_ok=True)
        sys.pycache_prefix = base  # type: ignore[attr-defined]
    except Exception:
        pass


_configure_pycache_prefix()

import numpy as np
import requests

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None  # type: ignore

try:
    import mediapipe as mp
except Exception:  # pragma: no cover
    mp = None  # type: ignore


# Pillow is optional; used for Chinese overlay text.
try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:  # pragma: no cover
    Image = None  # type: ignore
    ImageDraw = None  # type: ignore
    ImageFont = None  # type: ignore


@dataclass(frozen=True)
class JointScale:
    rad_min: float
    rad_max: float


def _unit(v: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.zeros_like(v)
    return v / n


def _angle_at(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """Angle ABC in radians."""
    ba = a - b
    bc = c - b
    ba_u = _unit(ba)
    bc_u = _unit(bc)
    dot = float(np.clip(np.dot(ba_u, bc_u), -1.0, 1.0))
    return float(math.acos(dot))


def _flexion_from_angle(angle_abc: float) -> float:
    """Convert internal joint angle to flexion measure.

    Straight finger typically yields angle ~ pi.
    Flexion = pi - angle, clamped >= 0.
    """
    return max(0.0, math.pi - float(angle_abc))


def _signed_angle_about_axis(v_from: np.ndarray, v_to: np.ndarray, axis: np.ndarray) -> float:
    """Signed angle from v_from to v_to about axis (radians)."""
    a = _unit(axis)
    v1 = _unit(v_from)
    v2 = _unit(v_to)
    cross = np.cross(v1, v2)
    sin_term = float(np.dot(a, cross))
    cos_term = float(np.clip(np.dot(v1, v2), -1.0, 1.0))
    return float(math.atan2(sin_term, cos_term))


def _clamp_to_scale(joint: str, value: float, scales: Dict[str, JointScale]) -> float:
    sc = scales.get(joint)
    if sc is None:
        return float(value)
    lo, hi = float(sc.rad_min), float(sc.rad_max)
    if lo <= hi:
        return float(np.clip(value, lo, hi))
    return float(np.clip(value, hi, lo))


def _map_flex_to_joint_range(flex_rad: float, joint: str, scales: Dict[str, JointScale]) -> float:
    """Map a flexion value to a joint command in radians.

    Heuristic:
    - Use joint_scale from config if present.
    - Treat flexion of ~90deg (pi/2) as "fully curled" (saturate).
    """
    sc = scales.get(joint)
    if sc is None:
        rad_min, rad_max = 0.0, 1.2
    else:
        rad_min, rad_max = float(sc.rad_min), float(sc.rad_max)

    if rad_max == rad_min:
        return float(rad_min)

    if rad_min >= 0.0:
        t = float(np.clip(flex_rad / (math.pi / 2.0), 0.0, 1.0))
        lo, hi = (rad_min, rad_max) if rad_min <= rad_max else (rad_max, rad_min)
        return float(np.clip(t * hi, lo, hi))

    t = float(np.clip(flex_rad / (math.pi / 2.0), -1.0, 1.0))
    return float(np.clip(t * max(abs(rad_min), abs(rad_max)), rad_min, rad_max))


def _ema_update(prev: Dict[str, float], cur: Dict[str, float], ema: float) -> Dict[str, float]:
    if not prev:
        return dict(cur)
    out: Dict[str, float] = dict(prev)
    for k, v in cur.items():
        if k in out:
            out[k] = float(ema) * float(out[k]) + (1.0 - float(ema)) * float(v)
        else:
            out[k] = float(v)
    return out


def _read_config(config_path: str) -> tuple[Dict[str, JointScale], Set[str]]:
    if yaml is None:
        return {}, set()

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            return {}, set()

        joint_to_servo_map = data.get("joint_to_servo_map", {})
        allowed_joints: Set[str] = set()
        if isinstance(joint_to_servo_map, dict):
            allowed_joints = {str(k) for k in joint_to_servo_map.keys()}

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


def _post_json(base: str, path: str, json_body):
    url = base.rstrip("/") + path
    r = requests.post(url, json=json_body, timeout=2.5)
    r.raise_for_status()
    return r.json()


def _get_json(base: str, path: str):
    url = base.rstrip("/") + path
    r = requests.get(url, timeout=2.5)
    r.raise_for_status()
    return r.json()


def _post_config_connect(base: str, config_path: str) -> None:
    url_config = base.rstrip("/") + "/config"
    r = requests.post(url_config, json=config_path, timeout=5)
    r.raise_for_status()

    url_connect = base.rstrip("/") + "/connect"
    try:
        r = requests.post(url_connect, json={}, timeout=5)
        r.raise_for_status()
    except Exception:
        # Best-effort: camera UI should still run even if the port is busy.
        return


def _get_overcurrent_from_version(base: str) -> tuple[bool, bool, Optional[float], list[int], dict[int, float]]:
    """Query API /version for connection + overcurrent status.

    Returns: (connected, enabled, default_limit_ma, tripped_ids, limit_by_id)
    """
    try:
        v = _get_json(base, "/version")
        if not isinstance(v, dict):
            return False, False, None, [], {}

        connected = bool(v.get("connected", False))
        enabled = bool(v.get("overcurrent_enabled", False))
        default_limit = v.get("overcurrent_limit_ma", None)
        try:
            default_limit_ma = None if default_limit is None else float(default_limit)
        except Exception:
            default_limit_ma = None
        raw_tripped = v.get("overcurrent_tripped_ids", [])
        tripped_ids: list[int] = []
        if isinstance(raw_tripped, list):
            for x in raw_tripped:
                try:
                    tripped_ids.append(int(x))
                except Exception:
                    continue
        raw_map = v.get("overcurrent_limit_ma_by_id", {})
        limit_by_id: dict[int, float] = {}
        if isinstance(raw_map, dict):
            for k, vv in raw_map.items():
                try:
                    sid = int(k)
                    ma = float(vv)
                    if ma > 0:
                        limit_by_id[int(sid)] = float(ma)
                except Exception:
                    continue
        return connected, enabled, default_limit_ma, tripped_ids, limit_by_id
    except Exception:
        return False, False, None, [], {}


def _post_overcurrent_reset(base: str, servo_ids: Optional[list[int]] = None) -> bool:
    try:
        body = {} if servo_ids is None else {"servo_ids": [int(x) for x in servo_ids]}
        _post_json(base, "/overcurrent/reset", body)
        return True
    except Exception:
        return False


def _extract_landmarks_3d(results, frame_w: int, frame_h: int) -> Optional[np.ndarray]:
    """Return landmarks as (21,3) in pseudo-3D.

    MediaPipe lm.z is roughly in the same normalized scale as x; scale it by frame_w.
    """
    if not results.multi_hand_landmarks:
        return None
    hand = results.multi_hand_landmarks[0]
    pts = []
    for lm in hand.landmark:
        pts.append([lm.x * frame_w, lm.y * frame_h, lm.z * frame_w])
    return np.asarray(pts, dtype=np.float32)


def _display_keys_all(allowed_joints: Set[str]) -> list[str]:
    ordered = [
        "拇指根部",
        "拇指侧摆",
        "拇指近端",
        "拇指远端",
        "食指侧摆",
        "食指近端",
        "食指远端",
        "中指侧摆",
        "中指近端",
        "中指远端",
        "无名指侧摆",
        "无名指近端",
        "无名指远端",
        "小指侧摆",
        "小指近端",
        "小指远端",
        "腕部关节",
    ]
    if allowed_joints:
        return [k for k in ordered if k in allowed_joints]
    return ordered


def _find_font_path(preferred: Optional[str] = None) -> Optional[str]:
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


def _draw_text_pil(
    frame_bgr: np.ndarray,
    lines: list[str] | list[tuple[str, tuple[int, int, int]]],
    font_path: Optional[str],
    font_size: int,
) -> np.ndarray:
    if Image is None or ImageDraw is None or ImageFont is None:
        return frame_bgr

    img = Image.fromarray(frame_bgr[:, :, ::-1])
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
            try:
                color = (int(bgr[2]), int(bgr[1]), int(bgr[0]))  # RGB for PIL
            except Exception:
                color = (255, 255, 255)
            draw.text((x, y), str(line), font=font, fill=color)
        else:
            draw.text((x, y), str(entry), font=font, fill=(255, 255, 255))
        y += line_h

    out = np.asarray(img)[:, :, ::-1]
    return out


def _draw_text_opencv(frame_bgr: np.ndarray, lines: list[str] | list[tuple[str, tuple[int, int, int]]]) -> np.ndarray:
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


def _compute_joint_commands_3d(
    pts: np.ndarray,
    scales: Dict[str, JointScale],
    allowed_joints: Set[str],
    enable_splay: Set[str],
    splay_invert: Set[str],
) -> Dict[str, float]:
    # Palm frame (pseudo-3D)
    wrist = pts[0]
    palm_x = _unit(pts[5] - pts[17])  # pinky -> index
    palm_y = _unit(pts[9] - wrist)  # wrist -> middle_mcp
    palm_z = _unit(np.cross(palm_x, palm_y))
    palm_y = _unit(np.cross(palm_z, palm_x))

    def flex_mcp(mcp: int, pip: int) -> float:
        return _flexion_from_angle(_angle_at(wrist, pts[mcp], pts[pip]))

    def flex_pip(mcp: int, pip: int, dip: int) -> float:
        return _flexion_from_angle(_angle_at(pts[mcp], pts[pip], pts[dip]))

    def flex_dip(pip: int, dip: int, tip: int) -> float:
        return _flexion_from_angle(_angle_at(pts[pip], pts[dip], pts[tip]))

    out: Dict[str, float] = {}

    # Four fingers: weighted MCP/PIP/DIP => prox/dist
    prox_w_mcp, prox_w_pip, prox_w_dip = 0.45, 0.45, 0.10
    dist_w_mcp, dist_w_pip, dist_w_dip = 0.10, 0.45, 0.45

    fingers = {
        "食指": (5, 6, 7, 8),
        "中指": (9, 10, 11, 12),
        "无名指": (13, 14, 15, 16),
        "小指": (17, 18, 19, 20),
    }
    for name, (mcp_i, pip_i, dip_i, tip_i) in fingers.items():
        f_mcp = flex_mcp(mcp_i, pip_i)
        f_pip = flex_pip(mcp_i, pip_i, dip_i)
        f_dip = flex_dip(pip_i, dip_i, tip_i)

        f_prox = prox_w_mcp * f_mcp + prox_w_pip * f_pip + prox_w_dip * f_dip
        f_dist = dist_w_mcp * f_mcp + dist_w_pip * f_pip + dist_w_dip * f_dip

        j_prox = f"{name}近端"
        j_dist = f"{name}远端"

        if (not allowed_joints) or (j_prox in allowed_joints):
            out[j_prox] = _clamp_to_scale(j_prox, _map_flex_to_joint_range(f_prox, j_prox, scales), scales)
        if (not allowed_joints) or (j_dist in allowed_joints):
            out[j_dist] = _clamp_to_scale(j_dist, _map_flex_to_joint_range(f_dist, j_dist, scales), scales)

    # Thumb: your mechanism-aligned mapping
    # - root(id4): metacarpal(1->2) rotation about a palm-tilted axis (~30deg to the left)
    # - splay(id3): MCP splay via (2->3) yaw in palm plane
    # - prox: MCP flexion (1-2-3)
    # - dist: IP/PIP flexion (2-3-4)
    tilt = math.radians(30.0)
    root_axis = _unit(math.cos(tilt) * palm_z + math.sin(tilt) * (-palm_x))

    thumb_meta = pts[2] - pts[1]
    ref = palm_y
    ref_p = ref - root_axis * float(np.dot(ref, root_axis))
    meta_p = thumb_meta - root_axis * float(np.dot(thumb_meta, root_axis))
    thumb_root_flex = abs(_signed_angle_about_axis(ref_p, meta_p, root_axis))

    thumb_mcp_flex = _flexion_from_angle(_angle_at(pts[1], pts[2], pts[3]))
    thumb_ip_flex = _flexion_from_angle(_angle_at(pts[2], pts[3], pts[4]))

    if (not allowed_joints) or ("拇指根部" in allowed_joints):
        out["拇指根部"] = _clamp_to_scale("拇指根部", _map_flex_to_joint_range(thumb_root_flex, "拇指根部", scales), scales)
    if (not allowed_joints) or ("拇指近端" in allowed_joints):
        out["拇指近端"] = _clamp_to_scale("拇指近端", _map_flex_to_joint_range(thumb_mcp_flex, "拇指近端", scales), scales)
    if (not allowed_joints) or ("拇指远端" in allowed_joints):
        out["拇指远端"] = _clamp_to_scale("拇指远端", _map_flex_to_joint_range(thumb_ip_flex, "拇指远端", scales), scales)

    thumb_splay = 0.0
    if "all" in enable_splay or "thumb" in enable_splay:
        v_thumb_prox = pts[3] - pts[2]
        v_thumb_prox_palm = v_thumb_prox - palm_z * float(np.dot(v_thumb_prox, palm_z))
        ref_palm = palm_y - palm_z * float(np.dot(palm_y, palm_z))
        thumb_splay = _signed_angle_about_axis(ref_palm, v_thumb_prox_palm, palm_z)
        if "all" in splay_invert or "thumb" in splay_invert:
            thumb_splay = -float(thumb_splay)
    if (not allowed_joints) or ("拇指侧摆" in allowed_joints):
        out["拇指侧摆"] = _clamp_to_scale("拇指侧摆", float(thumb_splay), scales)

    # Finger splay (3D): use projected MCP->PIP directions within palm plane, signed about palm normal
    def proj_palm(v: np.ndarray) -> np.ndarray:
        return v - palm_z * float(np.dot(v, palm_z))

    v_mid = proj_palm(pts[10] - pts[9])
    v_idx = proj_palm(pts[6] - pts[5])
    v_rng = proj_palm(pts[14] - pts[13])
    v_pky = proj_palm(pts[18] - pts[17])

    splay_index = 0.0
    splay_ring = 0.0
    splay_pinky = 0.0
    splay_middle = 0.0

    if "all" in enable_splay or "index" in enable_splay:
        splay_index = _signed_angle_about_axis(v_mid, v_idx, palm_z)
        if "all" in splay_invert or "index" in splay_invert:
            splay_index = -float(splay_index)
    if "all" in enable_splay or "ring" in enable_splay:
        splay_ring = _signed_angle_about_axis(v_mid, v_rng, palm_z)
        if "all" in splay_invert or "ring" in splay_invert:
            splay_ring = -float(splay_ring)
    if "all" in enable_splay or "pinky" in enable_splay:
        splay_pinky = _signed_angle_about_axis(v_mid, v_pky, palm_z)
        if "all" in splay_invert or "pinky" in splay_invert:
            splay_pinky = -float(splay_pinky)
    if "all" in enable_splay or "middle" in enable_splay:
        # Middle splay: absolute yaw of middle finger direction relative to palm forward
        splay_middle = _signed_angle_about_axis(palm_y, v_mid, palm_z)
        if "all" in splay_invert or "middle" in splay_invert:
            splay_middle = -float(splay_middle)

    if (not allowed_joints) or ("食指侧摆" in allowed_joints):
        out["食指侧摆"] = _clamp_to_scale("食指侧摆", float(splay_index), scales)
    if (not allowed_joints) or ("中指侧摆" in allowed_joints):
        out["中指侧摆"] = _clamp_to_scale("中指侧摆", float(splay_middle), scales)
    if (not allowed_joints) or ("无名指侧摆" in allowed_joints):
        out["无名指侧摆"] = _clamp_to_scale("无名指侧摆", float(splay_ring), scales)
    if (not allowed_joints) or ("小指侧摆" in allowed_joints):
        out["小指侧摆"] = _clamp_to_scale("小指侧摆", float(splay_pinky), scales)

    if (not allowed_joints) or ("腕部关节" in allowed_joints):
        # Wrist (proxy): MediaPipe Hands has no forearm, so we estimate a stable 1-DOF
        # wrist flex/extend from palm orientation relative to the camera.
        # We treat flex/extend as rotation about the palm's left-right axis (palm_x).
        # Measure how the palm normal (palm_z) rotates about palm_x relative to camera +Z.
        camera_z = np.array([0.0, 0.0, 1.0], dtype=float)
        axis = palm_x
        ref = camera_z - axis * float(np.dot(camera_z, axis))
        val = palm_z - axis * float(np.dot(palm_z, axis))
        if float(np.linalg.norm(ref)) < 1e-8 or float(np.linalg.norm(val)) < 1e-8:
            wrist_rad = 0.0
        else:
            wrist_rad = float(_signed_angle_about_axis(ref, val, axis))
        out["腕部关节"] = _clamp_to_scale("腕部关节", wrist_rad, scales)

    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="摄像头手势识别(3D几何) → 关节角 → 调用 py_action HTTP API")
    parser.add_argument("--base", default="http://127.0.0.1:8001", help="API base URL")
    parser.add_argument("--config", default="py_action/config_orca.yaml", help="API 配置文件路径")
    parser.add_argument("--camera", type=int, default=0, help="摄像头索引（默认 0）")
    parser.add_argument("--hz", type=float, default=10.0, help="发送频率 Hz")
    parser.add_argument("--ema", type=float, default=0.75, help="指数平滑系数(0..1)，越大越平滑")
    parser.add_argument(
        "--hand",
        choices=["auto", "left", "right"],
        default="auto",
        help="选择使用哪只手（auto/left/right）。实物是左手时建议用 left；auto 则用识别到的第一只手。",
    )
    parser.add_argument("--dry-run", action="store_true", help="不发送到 API，只做识别与显示")
    parser.add_argument("--no-window", action="store_true", help="不显示窗口（仅发送）")
    parser.add_argument(
        "--enable-splay",
        nargs="*",
        default=["all"],
        choices=["thumb", "index", "middle", "ring", "pinky", "all"],
        help=(
            "开启哪些手指的侧摆计算（默认 all：全开启）。"
            "可选：thumb/index/middle/ring/pinky/all。"
        ),
    )
    parser.add_argument(
        "--splay-invert",
        nargs="*",
        default=[],
        choices=["thumb", "index", "middle", "ring", "pinky", "all"],
        help=(
            "对指定手指的侧摆取反（用于校正方向）。"
            "可选：thumb/index/middle/ring/pinky/all。"
        ),
    )
    parser.add_argument(
        "--feedback",
        choices=["none", "motors", "joints"],
        default="none",
        help="可选：从 API 读回反馈并叠加显示（none/motors/joints）。注意：/motors/position 与 /joints/position 都是 GET 读回接口。",
    )
    parser.add_argument("--feedback-hz", type=float, default=5.0, help="反馈读取频率 Hz（默认 5Hz）")
    parser.add_argument(
        "--text-render",
        choices=["auto", "opencv", "pil"],
        default="auto",
        help="窗口文字渲染方式：opencv（快但中文会乱码）、pil（支持中文，需字体）、auto（检测到中文自动用 pil）。",
    )
    parser.add_argument(
        "--font",
        default=None,
        help=r"Pillow 渲染用字体文件路径（例如 C:\\Windows\\Fonts\\msyh.ttc）。不填则自动尝试系统中文字体。",
    )
    parser.add_argument("--font-size", type=int, default=20, help="Pillow 渲染字体大小（像素）")
    parser.add_argument(
        "--auto-reconnect",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="当 API 显示未连接时，后台自动尝试 POST /connect（默认开启）。",
    )
    args = parser.parse_args()

    if mp is None or cv2 is None:
        raise RuntimeError("缺少依赖：需要 mediapipe + opencv")

    enable_splay = {str(x).lower() for x in (args.enable_splay or [])}
    splay_invert = {str(x).lower() for x in (args.splay_invert or [])}

    scales, allowed_joints = _read_config(args.config)

    if not args.dry_run:
        _post_config_connect(args.base, args.config)

    cap = cv2.VideoCapture(int(args.camera))
    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头 index={args.camera}")

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    period = 1.0 / max(1e-6, float(args.hz))
    last_send = 0.0

    feedback_mode = str(args.feedback)
    feedback_period = 1.0 / max(1e-6, float(args.feedback_hz))
    feedback_last = 0.0

    oc_poll_period = 0.5
    oc_last = 0.0
    api_connected = False
    oc_enabled = False
    oc_default_limit = None
    oc_tripped: list[int] = []

    last_api_error: Optional[str] = None
    last_connect_try = 0.0

    prev: Dict[str, float] = {}

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        frame_h, frame_w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        pts = _extract_landmarks_3d(results, frame_w=frame_w, frame_h=frame_h)

        handed_label = None
        if pts is not None:
            try:
                if results.multi_handedness and results.multi_handedness[0].classification:
                    handed_label = results.multi_handedness[0].classification[0].label
            except Exception:
                handed_label = None

            if args.hand != "auto" and handed_label is not None:
                want = "Left" if args.hand == "left" else "Right"
                if handed_label != want:
                    pts = None

        overlay_lines: list[tuple[str, tuple[int, int, int]]] = []
        feedback_lines: list[tuple[str, tuple[int, int, int]]] = []

        # Poll overcurrent status (cheap, throttled)
        if (not args.dry_run) and (time.monotonic() - oc_last >= oc_poll_period):
            oc_last = time.monotonic()
            api_connected, oc_enabled, oc_default_limit, oc_tripped, _limit_by_id = _get_overcurrent_from_version(args.base)

            # Optional auto reconnect
            if bool(args.auto_reconnect) and (not api_connected) and (time.monotonic() - last_connect_try >= 2.0):
                last_connect_try = time.monotonic()
                try:
                    _post_json(args.base, "/connect", {})
                    last_api_error = None
                except Exception as e:
                    last_api_error = str(e)

        if pts is not None:
            # Keep the same mirroring behavior as the 2D script for consistency
            if handed_label == "Left":
                pts = pts.copy()
                pts[:, 0] = float(frame_w) - pts[:, 0]

            cur = _compute_joint_commands_3d(
                pts,
                scales,
                allowed_joints,
                enable_splay=enable_splay,
                splay_invert=splay_invert,
            )
            prev = _ema_update(prev, cur, float(args.ema))

            now = time.monotonic()
            if now - last_send >= period:
                last_send = now
                if not args.dry_run:
                    try:
                        # If any servo is tripped, pause sending new commands (safety-first).
                        if oc_tripped:
                            pass
                        elif not api_connected:
                            pass
                        else:
                            _post_json(args.base, "/joints/position", {"positions": prev})
                            last_api_error = None
                    except Exception as e:
                        # Keep a short last error for UI (avoid spamming terminal logs).
                        if isinstance(e, requests.HTTPError) and getattr(e, "response", None) is not None:
                            resp = e.response
                            detail = None
                            try:
                                j = resp.json()
                                if isinstance(j, dict):
                                    detail = j.get("detail")
                            except Exception:
                                detail = None
                            last_api_error = f"HTTP {resp.status_code}: {detail or resp.text}"
                        else:
                            last_api_error = str(e)

            if (not args.dry_run) and feedback_mode != "none" and (now - feedback_last >= feedback_period):
                feedback_last = now
                try:
                    if feedback_mode == "motors":
                        fb = _get_json(args.base, "/motors/position")
                        pos = fb.get("positions")
                        if isinstance(pos, list) and len(pos) >= 1:
                            feedback_lines.append((f"FB ID1 raw: {pos[0]}", (180, 180, 180)))
                    elif feedback_mode == "joints":
                        fb = _get_json(args.base, "/joints/position")
                        jp = fb.get("positions")
                        if isinstance(jp, dict):
                            v = jp.get("拇指近端")
                            if v is not None:
                                feedback_lines.append((f"FB 拇指近端: {float(v):+.3f} rad", (180, 180, 180)))
                except Exception:
                    pass

            show_keys = _display_keys_all(allowed_joints)
            overlay_lines.append(("[3D] joints (rad)", (255, 255, 255)))
            for k in show_keys:
                v = prev.get(k)
                if v is None:
                    continue
                overlay_lines.append((f"{k}: {float(v):+.3f}", (255, 255, 255)))
            overlay_lines.extend(feedback_lines)

            # Over-current status UI
            if not args.dry_run:
                if not api_connected:
                    overlay_lines.append(("[API] not connected (sending paused)", (0, 0, 255)))
                    if last_api_error:
                        overlay_lines.append((f"[API] last error: {last_api_error}", (0, 0, 255)))

                if oc_enabled:
                    if oc_default_limit is None:
                        overlay_lines.append(("[OC] enabled", (180, 180, 180)))
                    else:
                        overlay_lines.append((f"[OC] enabled default={float(oc_default_limit):.0f}mA", (180, 180, 180)))
                else:
                    overlay_lines.append(("[OC] disabled", (120, 120, 120)))

                if oc_tripped:
                    overlay_lines.append((f"[OC] TRIPPED IDs: {', '.join(str(i) for i in oc_tripped)} (sending paused)", (0, 0, 255)))
                    overlay_lines.append(("[OC] press 'r' to reset trips", (0, 0, 255)))

        else:
            # Still render UI so the window doesn't look frozen when no hand is detected.
            overlay_lines.append(("[3D] No hand detected", (255, 255, 255)))
            if args.hand != "auto" and handed_label is not None:
                overlay_lines.append((f"Detected: {handed_label} (filtered by --hand {args.hand})", (180, 180, 180)))

            if (not args.dry_run) and oc_tripped:
                overlay_lines.append((f"[OC] TRIPPED IDs: {', '.join(str(i) for i in oc_tripped)} (sending paused)", (0, 0, 255)))
                overlay_lines.append(("[OC] press 'r' to reset trips", (0, 0, 255)))

        if not args.no_window:
            if results.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)

            render = str(args.text_render)
            if render == "auto":
                def _line_text(entry) -> str:
                    if isinstance(entry, tuple) and len(entry) == 2:
                        return str(entry[0])
                    return str(entry)

                render = "pil" if any(any(ord(ch) > 127 for ch in _line_text(s)) for s in overlay_lines) else "opencv"

            if render == "pil":
                frame = _draw_text_pil(frame, overlay_lines, font_path=args.font, font_size=int(args.font_size))
            else:
                frame = _draw_text_opencv(frame, overlay_lines)

            cv2.imshow("camera3d_hand_to_api", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if (not args.dry_run) and key in (ord("r"), ord("R")):
                _post_overcurrent_reset(args.base, None)
                # refresh soon
                oc_last = 0.0

    cap.release()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
