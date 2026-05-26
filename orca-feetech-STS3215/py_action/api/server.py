from __future__ import annotations

import os
import sys
import time
import atexit
import threading
import tempfile
from dataclasses import dataclass, field
from contextlib import asynccontextmanager
import logging
from typing import Dict, List, Optional, Union, Tuple, Any

from fastapi import FastAPI, HTTPException, Body, Request
from pydantic import BaseModel, Field
import uvicorn


def _ocra_temp_dir() -> str:
    return os.path.join(tempfile.gettempdir(), "ocra_py_action")


def _configure_pycache_prefix() -> None:
    # Centralize bytecode cache so we don't create __pycache__ in every folder.
    try:
        base = os.environ.get("OCRA_PYCACHE_DIR") or os.path.join(_ocra_temp_dir(), "pycache")
        os.makedirs(base, exist_ok=True)
        sys.pycache_prefix = base  # type: ignore[attr-defined]
    except Exception:
        pass


def _bootstrap_module_search_path() -> None:
    # We intentionally avoid package imports (no reliance on __init__.py).
    api_dir = os.path.abspath(os.path.dirname(__file__))
    py_action_root = os.path.abspath(os.path.join(api_dir, os.pardir))
    workspace_root = os.path.abspath(os.path.join(py_action_root, os.pardir))

    paths = [
        workspace_root,
        py_action_root,
        api_dir,
        os.path.join(py_action_root, "control"),
        os.path.join(py_action_root, "hardware"),
        os.path.join(py_action_root, "vision"),
        os.path.join(py_action_root, "gui"),
    ]
    for p in reversed(paths):
        if p not in sys.path:
            sys.path.insert(0, p)


_configure_pycache_prefix()
_bootstrap_module_search_path()

import actions

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover
    yaml = None

try:
    from serial.tools import list_ports  # type: ignore
except Exception:  # pragma: no cover
    list_ports = None


logger = logging.getLogger("ocra_py_action.api.server")


def _best_effort_release_serial(reason: str, *, no_zero: bool = False) -> None:
    """Try to release the serial port so the next run can connect.

    This runs during shutdown/reload; it must never raise.
    """

    try:
        if not actions.is_connected():
            return
        if not no_zero:
            try:
                # best-effort stop
                if hasattr(actions, "emergency_stop"):
                    actions.emergency_stop()  # type: ignore[attr-defined]
                    time.sleep(0.05)
            except Exception:
                pass

        try:
            actions.disconnect()
        except Exception:
            pass

        logger.info("Serial released (%s)", reason)
    except Exception:
        pass


atexit.register(lambda: _best_effort_release_serial("atexit"))


@asynccontextmanager
async def lifespan(_app: FastAPI):
    yield
    _best_effort_release_serial("lifespan shutdown")


app = FastAPI(
    title="OrcaHand 控制 API",
    version="1.0.0",
    description=(
        "用于驱动灵巧手（SMS/STS 舵机）的 HTTP API。\n\n"
        "典型调用顺序：POST /config → POST /connect → POST /joints/position。"
    ),
    lifespan=lifespan,
)


class MessageResponse(BaseModel):
    message: str = Field(..., description="提示信息")


class VersionResponse(BaseModel):
    name: str = Field(..., description="服务名称")
    version: str = Field(..., description="服务版本")
    pid: int = Field(..., description="当前进程 PID")
    config_path: Optional[str] = Field(None, description="当前加载的配置文件路径（绝对路径）")
    connected: bool = Field(..., description="是否已连接串口/灵巧手")
    effective_port: Optional[str] = Field(
        None,
        description="当前实际使用的串口（优先级：已加载配置 > 环境变量 OCRA_SERIAL_PORT；未配置则为 null）。",
    )
    effective_baudrate: Optional[int] = Field(
        None,
        description="当前实际使用的波特率（已加载配置时返回；否则可能为 null）。",
    )

    overcurrent_enabled: bool = Field(False, description="是否启用过流保护（基于电流寄存器反馈）。")
    overcurrent_limit_ma: Optional[float] = Field(None, description="过流阈值（mA）；未启用时为 null。")
    overcurrent_limit_ma_by_id: Dict[int, float] = Field(
        default_factory=dict,
        description="可选：每个舵机的独立过流阈值（mA）。key=舵机ID。",
    )
    overcurrent_tripped_ids: List[int] = Field(default_factory=list, description="已触发保护（被熔断）的舵机 ID 列表。")


@app.get(
    "/version",
    summary="服务版本/实例信息",
    description="用于确认当前访问到的是哪个运行实例（包含 PID、配置路径、连接状态）。",
    tags=["状态"],
    response_model=VersionResponse,
)
def get_version():
    try:
        effective_port: Optional[str] = None
        effective_baudrate: Optional[int] = None
        if _config is not None:
            effective_port = str(_config.port)
            effective_baudrate = int(_config.baudrate)
        else:
            env_port = str(os.environ.get("OCRA_SERIAL_PORT", "") or "").strip()
            if env_port:
                effective_port = env_port
        oc = {}
        try:
            if hasattr(actions, "get_overcurrent_status"):
                oc = actions.get_overcurrent_status()  # type: ignore[attr-defined]
        except Exception:
            oc = {}

        return {
            "name": app.title,
            "version": app.version,
            "pid": os.getpid(),
            "config_path": _current_config_path,
            "connected": actions.is_connected(),
            "effective_port": effective_port,
            "effective_baudrate": effective_baudrate,
            "overcurrent_enabled": bool(oc.get("enabled", False)),
            "overcurrent_limit_ma": oc.get("limit_ma", None),
            "overcurrent_limit_ma_by_id": dict(oc.get("limit_ma_by_id", {}) or {}),
            "overcurrent_tripped_ids": list(oc.get("tripped_ids", []) or []),
        }
    except Exception as e:
        _handle_exception(e)


@app.post(
    "/shutdown",
    summary="停止服务（仅本机）",
    description=(
        "优雅退出兜底接口：当你误关了启动 uvicorn 的终端窗口，但服务仍在后台运行时使用。\n\n"
        "限制：只允许从 localhost 调用（127.0.0.1 / ::1）。\n\n"
        "可选：传 {\"no_zero\": true} 表示关闭时不执行回零（仅断开串口/释放资源）。"
    ),
    tags=["管理"],
    response_model=MessageResponse,
)
def shutdown_server(request: Request, payload: Dict[str, Any] = Body(default_factory=dict)):
    """Shuts down the current process.

    Notes:
    - Only allowed from localhost.
    - Useful when the terminal window was closed but the server is still running.
    """

    host = getattr(request.client, "host", "") if request.client else ""
    if host not in ("127.0.0.1", "::1"):
        _http_error(403, "Shutdown is only allowed from localhost")

    # Allow caller to request "no physical zero" shutdown.
    no_zero = False
    try:
        v = payload.get("no_zero") if isinstance(payload, dict) else None
        if isinstance(v, str):
            no_zero = v.strip().lower() in ("1", "true", "yes", "y", "on")
        else:
            no_zero = bool(v)
    except Exception:
        no_zero = False

    _best_effort_release_serial("api /shutdown", no_zero=no_zero)

    def _exit_soon():
        time.sleep(0.2)
        os._exit(0)

    threading.Thread(target=_exit_soon, daemon=True).start()
    return {"message": "Server shutting down"}


# -----------------------------
# Models (match orca_core API)
# -----------------------------
class MotorList(BaseModel):
    motor_ids: Optional[List[int]] = Field(
        default=None,
        description="要操作的电机 ID 列表；为空/不填表示默认范围（兼容原版接口）。",
        examples=[[1, 2, 3]],
    )


class MaxCurrent(BaseModel):
    current: Union[float, List[float]] = Field(
        ...,
        description="最大电流值（兼容字段）。SMS/STS 后端当前为 no-op。",
        examples=[0.5, [0.5, 0.6, 0.7]],
    )


class JointPositions(BaseModel):
    positions: Dict[str, float] = Field(
        ...,
        description=(
            "关节目标角度（单位：弧度 rad）。key 必须与配置文件 joint_to_servo_map 的关节名一致（本项目通常是中文关节名）。"
        ),
        examples=[{"食指近端": 0.5, "拇指近端": 0.2}],
    )


class PortsResponse(BaseModel):
    ports: List[str] = Field(default_factory=list, description="当前机器检测到的串口列表（如 COM3/COM5）。")


class StatusResponse(BaseModel):
    connected: bool = Field(..., description="是否已连接串口/灵巧手")
    calibrated: bool = Field(
        ...,
        description="是否已准备好关节映射（此处 calibrated 表示映射配置可用，并非真实硬件标定状态）。",
    )


class MotorPositionsResponse(BaseModel):
    positions: Optional[List[Optional[int]]] = Field(
        None,
        description="电机 raw 位置列表（按 1..17 顺序）；未连接时为 null；单个电机读不到则为 null。",
        examples=[[2047, 2047, None]],
    )
    unit: str = Field("raw", description="单位标识；当前为 raw（0..4095）。")


class JointPositionsResponse(BaseModel):
    positions: Optional[Dict[str, Optional[float]]] = Field(
        None,
        description="关节角度映射（单位：rad）；未连接时为 null；读不到则值为 null。",
        examples=[{"食指近端": 0.1, "拇指近端": None}],
    )
    unit: str = Field("radians", description="单位标识；当前为 radians（弧度）。")


class CalibrationStatusResponse(BaseModel):
    calibrated: bool = Field(
        ...,
        description="映射配置是否就绪（同 /status.calibrated 语义）。",
    )


class CalibrateResponse(BaseModel):
    message: str = Field(..., description="提示信息")
    calibrated: bool = Field(..., description="是否完成标定（SMS/STS 后端当前恒为 false）。")


# -----------------------------
# Config & mapping
# -----------------------------
@dataclass
class JointScale:
    rad_min: float
    rad_max: float
    pos_min: int
    pos_max: int


@dataclass
class ApiConfig:
    port: str
    baudrate: int = 1_000_000
    joint_to_servo_map: Dict[str, int] = field(default_factory=dict)
    joint_scale: Dict[str, JointScale] = field(default_factory=dict)

    # Over-current protection (optional)
    overcurrent_enabled: bool = False
    overcurrent_limit_ma: Optional[float] = None
    overcurrent_limit_ma_by_id: Dict[int, float] = field(default_factory=dict)
    overcurrent_latch: bool = True
    overcurrent_reset_ratio: float = 0.80


_current_config_path: Optional[str] = None
_config: Optional[ApiConfig] = None


def _http_error(status_code: int, detail: str):
    raise HTTPException(status_code=status_code, detail=detail)


def _handle_exception(e: Exception):
    # Keep semantics close to orca_core: 409 for "not connected"-like states.
    msg = str(e)
    try:
        logger.exception("API error: %s", e)
    except Exception:
        pass
    if isinstance(e, RuntimeError) and "not connected" in msg.lower():
        _http_error(409, f"Hand operation failed: {e}")
    if isinstance(e, ValueError):
        _http_error(422, f"Invalid input: {e}")
    _http_error(500, f"Internal server error: {e}")


def _is_serial_open_like_error(e: Exception) -> bool:
    # pyserial errors may come wrapped in SerialException; match by message and Windows winerror.
    msg = str(e).lower()
    winerror = getattr(e, "winerror", None)
    if winerror in (5, 31, 32):
        return True
    if "cannot configure port" in msg or "access is denied" in msg:
        return True
    if "设备没有发挥作用" in str(e) or "拒绝访问" in str(e):
        return True
    return isinstance(e, PermissionError)


def _require_config() -> ApiConfig:
    if _config is None:
        _http_error(
            409,
            "No configuration loaded. Call POST /config with a config file path first.",
        )
    assert _config is not None
    return _config


def _load_yaml(path: str) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is not installed. Please `pip install pyyaml`.")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("config yaml root must be a mapping")
    return data


def _parse_joint_scale(obj: Any) -> JointScale:
    if not isinstance(obj, dict):
        raise ValueError("joint_scale entry must be an object")
    return JointScale(
        rad_min=float(obj["rad_min"]),
        rad_max=float(obj["rad_max"]),
        pos_min=int(obj["pos_min"]),
        pos_max=int(obj["pos_max"]),
    )


def _load_config_from_path(config_path: str) -> ApiConfig:
    data = _load_yaml(config_path)

    port = str(data.get("port", "") or "").strip()
    if port.upper() == "AUTO":
        port = ""

    # Optional override (useful when GUI selects the real port, but you don't want
    # to edit YAML or keep COMx hardcoded in config files).
    env_port = str(os.environ.get("OCRA_SERIAL_PORT", "") or "").strip()
    if env_port:
        port = env_port

    if not port:
        raise ValueError("config must include 'port' (e.g. COM5), or set env OCRA_SERIAL_PORT=COMx")

    try:
        baudrate = int(os.environ.get("OCRA_BAUDRATE", str(data.get("baudrate", 1_000_000))))
    except Exception:
        baudrate = int(data.get("baudrate", 1_000_000))

    j2s = data.get("joint_to_servo_map", {})
    if not isinstance(j2s, dict):
        raise ValueError("joint_to_servo_map must be a mapping")
    joint_to_servo_map = {str(k): int(v) for k, v in j2s.items()}
    if not joint_to_servo_map:
        raise ValueError(
            "joint_to_servo_map is empty; please check your YAML (config_orca.yaml) and make sure it was loaded correctly"
        )

    scale_raw = data.get("joint_scale", {})
    if not isinstance(scale_raw, dict):
        raise ValueError("joint_scale must be a mapping")
    joint_scale = {str(k): _parse_joint_scale(v) for k, v in scale_raw.items()}

    # ---- Over-current protection (optional) ----
    oc_enabled = False
    oc_limit_ma: Optional[float] = None
    oc_limit_ma_by_id: Dict[int, float] = {}
    oc_latch = True
    oc_reset_ratio = 0.80

    # env override wins
    env_oc = str(os.environ.get("OCRA_OVERCURRENT_MA", "") or "").strip()
    if env_oc:
        try:
            oc_limit_ma = float(env_oc)
            oc_enabled = True
        except Exception:
            pass

    oc_cfg = data.get("overcurrent_protection", None)
    if isinstance(oc_cfg, dict):
        try:
            if oc_limit_ma is None:
                v = oc_cfg.get("default_limit_ma", None)
                if v is None:
                    v = oc_cfg.get("limit_ma", None)
                if v is not None:
                    oc_limit_ma = float(v)
            if not env_oc:
                oc_enabled = bool(oc_cfg.get("enabled", False))
        except Exception:
            pass
        try:
            raw = oc_cfg.get("per_servo_limit_ma", None)
            if isinstance(raw, dict):
                for k, v in raw.items():
                    try:
                        sid = int(k)
                        ma = float(v)
                        if 0 <= sid <= 253 and ma > 0:
                            oc_limit_ma_by_id[int(sid)] = float(ma)
                    except Exception:
                        continue
        except Exception:
            oc_limit_ma_by_id = {}
        try:
            oc_latch = bool(oc_cfg.get("latch", True))
        except Exception:
            oc_latch = True
        try:
            oc_reset_ratio = float(oc_cfg.get("reset_ratio", 0.80))
        except Exception:
            oc_reset_ratio = 0.80

    # Back-compat: allow flat keys
    if oc_limit_ma is None:
        v = data.get("overcurrent_limit_ma", None)
        if v is not None:
            try:
                oc_limit_ma = float(v)
            except Exception:
                oc_limit_ma = None

    if (oc_limit_ma is not None) and (not env_oc) and ("overcurrent_enabled" in data):
        try:
            oc_enabled = bool(data.get("overcurrent_enabled"))
        except Exception:
            pass

    # Apply to runtime (best-effort). Keep it here so GUI-started API gets it automatically.
    try:
        if hasattr(actions, "configure_overcurrent_protection"):
            actions.configure_overcurrent_protection(
                enabled=bool(oc_enabled),
                limit_ma=oc_limit_ma,
                limit_ma_by_id=(oc_limit_ma_by_id or None),
                latch=bool(oc_latch),
                reset_ratio=float(oc_reset_ratio),
            )
    except Exception:
        pass

    return ApiConfig(
        port=port,
        baudrate=baudrate,
        joint_to_servo_map=joint_to_servo_map,
        joint_scale=joint_scale,
        overcurrent_enabled=bool(oc_enabled),
        overcurrent_limit_ma=oc_limit_ma,
        overcurrent_limit_ma_by_id=(oc_limit_ma_by_id or {}),
        overcurrent_latch=bool(oc_latch),
        overcurrent_reset_ratio=float(oc_reset_ratio),
    )


class OvercurrentResetRequest(BaseModel):
    servo_ids: Optional[List[int]] = Field(
        default=None,
        description="要复位的舵机 ID 列表；为空/不填表示复位全部。",
        examples=[[3, 7, 12]],
    )


@app.post(
    "/overcurrent/reset",
    summary="复位过流熔断状态",
    description=(
        "清除已触发过流保护（tripped）的舵机列表，并 best-effort 重新使能 torque。\n\n"
        "- 不传 servo_ids：复位全部\n"
        "- 传 servo_ids：仅复位指定 ID"
    ),
    tags=["安全"],
    response_model=MessageResponse,
)
def overcurrent_reset(payload: OvercurrentResetRequest = Body(default_factory=OvercurrentResetRequest)):
    try:
        ids = payload.servo_ids
        if hasattr(actions, "reset_overcurrent_trips"):
            actions.reset_overcurrent_trips(ids)  # type: ignore[attr-defined]
        return {"message": "ok"}
    except Exception as e:
        _handle_exception(e)


def _clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _rad_to_servo_pos(joint: str, rad: float, cfg: ApiConfig) -> int:
    if cfg.joint_scale is None or joint not in cfg.joint_scale:
        raise ValueError(f"No joint_scale configured for joint '{joint}'")

    sc = cfg.joint_scale[joint]
    if sc.rad_max == sc.rad_min:
        raise ValueError(f"Invalid scale for joint '{joint}': rad_max == rad_min")

    rad_clipped = _clip(float(rad), sc.rad_min, sc.rad_max)
    t = (rad_clipped - sc.rad_min) / (sc.rad_max - sc.rad_min)
    pos = sc.pos_min + t * (sc.pos_max - sc.pos_min)
    return int(round(_clip(pos, 0, 4095)))


def _servo_positions_from_joint_dict(joint_dict: Dict[str, float], cfg: ApiConfig) -> List[Tuple[int, int]]:
    if not cfg.joint_to_servo_map:
        raise ValueError("joint_to_servo_map is empty; cannot map joint names to servo IDs")

    moves: List[Tuple[int, int]] = []
    for joint, rad in joint_dict.items():
        if joint not in cfg.joint_to_servo_map:
            raise ValueError(f"Unknown joint '{joint}' (missing in joint_to_servo_map)")
        sid = int(cfg.joint_to_servo_map[joint])
        pos = _rad_to_servo_pos(joint, float(rad), cfg)
        moves.append((sid, pos))

    return moves


# -----------------------------
# Endpoints (match orca_core)
# -----------------------------
@app.post(
    "/config",
    summary="加载配置文件",
    description=(
        "从 YAML 加载配置：串口号/波特率、关节名→舵机 ID 映射、rad↔raw 线性映射。\n\n"
        "注意：该接口会 best-effort 断开当前连接（如已连接），然后再更新配置。"
    ),
    tags=["配置"],
    response_model=MessageResponse,
)
def set_hand_config(
    config_path: str = Body(
        ...,
        description="配置文件路径（建议使用工作区相对路径，例如 py_action/config_orca.yaml）",
        examples=["py_action/config_orca.yaml"],
    )
):
    global _config, _current_config_path
    try:
        def _workspace_root_dir() -> str:
            # py_action/api/server.py -> workspace root (ws)
            return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))

        # Resolve relative paths robustly even if cwd is py_action/ or elsewhere.
        resolved = os.path.abspath(config_path)
        if not os.path.exists(resolved):
            wr = _workspace_root_dir()
            cand = os.path.abspath(os.path.join(wr, config_path))
            if os.path.exists(cand):
                resolved = cand
            else:
                if not str(config_path).replace("\\", "/").startswith("py_action/"):
                    cand2 = os.path.abspath(os.path.join(wr, "py_action", config_path))
                    if os.path.exists(cand2):
                        resolved = cand2

        if not os.path.exists(resolved):
            _http_error(404, f"Config file not found: {resolved}")

        # best-effort: disconnect before switching config
        try:
            if actions.is_connected():
                actions.disconnect()
                time.sleep(0.05)
        except Exception:
            pass

        _config = _load_config_from_path(resolved)
        _current_config_path = resolved
        return {"message": f"Hand configuration updated to: {config_path}"}
    except HTTPException:
        raise
    except Exception as e:
        _handle_exception(e)


@app.get(
    "/serial/ports",
    summary="列出本机串口",
    description="列出当前机器可用的串口设备（用于选择 COM 口）。",
    tags=["配置"],
    response_model=PortsResponse,
)
def get_serial_ports():
    if list_ports is None:
        return {"ports": []}
    return {"ports": [p.device for p in list_ports.comports()]}


@app.post(
    "/connect",
    summary="连接灵巧手",
    description="打开串口并建立与灵巧手的通信连接。",
    tags=["连接"],
    response_model=MessageResponse,
)
def connect_hand():
    if actions.is_connected():
        return {"message": "Hand already connected."}

    cfg = _require_config()
    try:
        actions.connect(cfg.port, baudrate=cfg.baudrate)
        return {"message": "Connection successful"}
    except Exception as e:
        if _is_serial_open_like_error(e):
            _http_error(
                409,
                (
                    f"Cannot open/configure serial port {cfg.port} (baudrate={cfg.baudrate}). "
                    "请检查：1) 设备是否上电/已插入；2) 该 COM 是否被其它程序占用（包括本 GUI 的‘开始调试’）；"
                    "3) 设备管理器里端口是否正常；4) 重新插拔/更换 USB 线或驱动。 "
                    f"Original: {e}"
                ),
            )
        _handle_exception(e)


@app.post(
    "/disconnect",
    summary="断开连接",
    description="急停（best-effort）并断开串口连接，释放串口占用。",
    tags=["连接"],
    response_model=MessageResponse,
)
def disconnect_hand():
    if not actions.is_connected():
        return {"message": "Hand already disconnected."}

    try:
        # best-effort stop
        try:
            actions.emergency_stop()
            time.sleep(0.05)
        except Exception:
            pass

        actions.disconnect()
        return {"message": "Disconnected successfully"}
    except Exception as e:
        _handle_exception(e)


@app.get(
    "/status",
    summary="获取状态",
    description="返回连接状态以及映射配置是否就绪（calibrated=映射就绪）。",
    tags=["状态"],
    response_model=StatusResponse,
)
def get_status():
    try:
        # Here "calibrated" means: mapping is ready for joint control.
        calibrated = bool(_config and _config.joint_to_servo_map and _config.joint_scale)
        return {"connected": actions.is_connected(), "calibrated": calibrated}
    except Exception as e:
        _handle_exception(e)


@app.post(
    "/torque/enable",
    summary="开启扭矩（兼容接口）",
    description="兼容原版接口：SMS/STS 后端当前未实现（no-op）。",
    tags=["控制"],
    response_model=MessageResponse,
)
def enable_torque(_motor_list: MotorList = Body(None)):
    if not actions.is_connected():
        _http_error(409, "Hand operation failed: not connected")
    return {"message": "Torque enable not implemented for SMS/STS backend (no-op)."}


@app.post(
    "/torque/disable",
    summary="关闭扭矩（兼容接口）",
    description="兼容原版接口：SMS/STS 后端当前未实现（no-op）。",
    tags=["控制"],
    response_model=MessageResponse,
)
def disable_torque(_motor_list: MotorList = Body(None)):
    if not actions.is_connected():
        _http_error(409, "Hand operation failed: not connected")
    return {"message": "Torque disable not implemented for SMS/STS backend (no-op)."}


@app.post(
    "/current/max",
    summary="设置最大电流（兼容接口）",
    description="兼容原版接口：SMS/STS 后端当前未实现（no-op）。",
    tags=["控制"],
    response_model=MessageResponse,
)
def set_max_current(_max_current: MaxCurrent):
    if not actions.is_connected():
        _http_error(409, "Hand operation failed: not connected")
    return {"message": "Max current not implemented for SMS/STS backend (no-op)."}


@app.get(
    "/motors/position",
    summary="读取电机 raw 位置",
    description="读取 1..17 号舵机 raw 位置（0..4095）。未连接时返回 null。",
    tags=["状态读取"],
    response_model=MotorPositionsResponse,
)
def get_motor_position():
    if not actions.is_connected():
        return {"positions": None}

    try:
        with actions.bus_lock:
            actions._require_bus()  # type: ignore[attr-defined]
            raw = actions.bus.sync_read(range(1, 18), start_addr=0x38, data_len=2)

        # keep list order 1..17
        positions: List[Optional[int]] = []
        for sid in range(1, 18):
            b = raw.get(sid)
            if not b or len(b) != 2:
                positions.append(None)
            else:
                positions.append(int(b[0] | (b[1] << 8)))

        return {"positions": positions, "unit": "raw"}
    except Exception as e:
        _handle_exception(e)


@app.get(
    "/joints/position",
    summary="读取关节角度",
    description=(
        "在配置存在 joint_to_servo_map + joint_scale 时，将 raw 位置反算为关节弧度(rad)返回。\n\n"
        "注意：SMS/STS 后端只能读 raw；反算依赖配置中的 joint_scale。"
    ),
    tags=["状态读取"],
    response_model=JointPositionsResponse,
)
def get_joint_position():
    if not actions.is_connected():
        return {"positions": None}

    cfg = _require_config()
    if not cfg.joint_to_servo_map:
        return {"positions": {}}

    # We can only read raw servo positions; converting back to radians requires joint_scale.
    if not cfg.joint_scale:
        return {"positions": {}}

    try:
        servo_ids = sorted(set(int(v) for v in cfg.joint_to_servo_map.values()))
        with actions.bus_lock:
            actions._require_bus()  # type: ignore[attr-defined]
            raw = actions.bus.sync_read(servo_ids, start_addr=0x38, data_len=2)

        # Build reverse map sid -> joints using it
        sid_to_joints: Dict[int, List[str]] = {}
        for j, sid in cfg.joint_to_servo_map.items():
            sid_to_joints.setdefault(int(sid), []).append(j)

        out: Dict[str, Optional[float]] = {}
        for sid, joints in sid_to_joints.items():
            b = raw.get(sid)
            if not b or len(b) != 2:
                for j in joints:
                    out[j] = None
                continue
            pos = int(b[0] | (b[1] << 8))
            for j in joints:
                sc = cfg.joint_scale.get(j)
                if sc is None or sc.pos_max == sc.pos_min:
                    out[j] = None
                    continue
                t = (pos - sc.pos_min) / float(sc.pos_max - sc.pos_min)
                rad = sc.rad_min + t * (sc.rad_max - sc.rad_min)
                out[j] = float(rad)

        return {"positions": out, "unit": "radians"}
    except Exception as e:
        _handle_exception(e)


@app.post(
    "/joints/position",
    summary="设置关节目标角度",
    description=(
        "按关节名下发目标角度（rad），服务端会根据配置映射为舵机 raw 位置并同步下发。\n\n"
        "前置条件：已 POST /config 且已 POST /connect。"
    ),
    tags=["控制"],
    response_model=MessageResponse,
)
def set_joint_position(joint_positions: JointPositions):
    if not actions.is_connected():
        _http_error(409, "Hand operation failed: not connected")

    cfg = _require_config()
    try:
        if actions.is_action_running():
            _http_error(409, "Hand operation failed: action thread is running")

        moves = _servo_positions_from_joint_dict(joint_positions.positions, cfg)
        actions.move_simultaneous(moves)
        return {"message": "Joint positions command sent successfully."}
    except HTTPException:
        raise
    except Exception as e:
        _handle_exception(e)


@app.get(
    "/calibrate/status",
    summary="标定状态（映射就绪）",
    description="返回映射配置是否就绪（同 /status.calibrated 语义）。",
    tags=["标定"],
    response_model=CalibrationStatusResponse,
)
def get_calibration_status():
    calibrated = bool(_config and _config.joint_to_servo_map and _config.joint_scale)
    return {"calibrated": calibrated}


@app.post(
    "/calibrate",
    summary="自动标定（兼容接口）",
    description="兼容原版接口：SMS/STS 后端当前未实现自动标定（no-op）。",
    tags=["标定"],
    response_model=CalibrateResponse,
)
def calibrate_auto():
    # Not implemented for SMS/STS: keep endpoint for compatibility.
    if not actions.is_connected():
        _http_error(409, "Hand must be connected to calibrate.")
    return {"message": "Calibration not implemented for SMS/STS backend.", "calibrated": False}


if __name__ == "__main__":
    host = os.environ.get("OCRA_API_HOST", "127.0.0.1")
    try:
        port = int(os.environ.get("OCRA_API_PORT", "8001"))
    except Exception:
        port = 8001
    uvicorn.run(app, host=host, port=port)
