import tkinter as tk
from tkinter import messagebox
import os
import sys
import subprocess
import queue
import signal
import threading
import time
import tempfile
import importlib
import math
from typing import Optional
import traceback


def _ocra_temp_dir() -> str:
    return os.path.join(tempfile.gettempdir(), "ocra_py_action")


def _configure_pycache_prefix() -> None:
    # Centralize bytecode cache so we don't create __pycache__ in every folder.
    # Works in CPython 3.8+ via sys.pycache_prefix.
    try:
        base = os.environ.get("OCRA_PYCACHE_DIR") or os.path.join(_ocra_temp_dir(), "pycache")
        os.makedirs(base, exist_ok=True)
        sys.pycache_prefix = base  # type: ignore[attr-defined]
    except Exception:
        pass


def _bootstrap_module_search_path() -> None:
    # We intentionally avoid package imports (no reliance on __init__.py).
    # When running this file directly (python py_action/gui/app.py), sys.path[0]
    # becomes py_action/gui.
    gui_dir = os.path.abspath(os.path.dirname(__file__))
    py_action_root = os.path.abspath(os.path.join(gui_dir, os.pardir))
    workspace_root = os.path.abspath(os.path.join(py_action_root, os.pardir))

    paths = [
        workspace_root,
        py_action_root,
        os.path.join(py_action_root, "api"),
        os.path.join(py_action_root, "control"),
        os.path.join(py_action_root, "hardware"),
        os.path.join(py_action_root, "vision"),
        os.path.join(py_action_root, "gui"),
    ]

    # Insert in reverse so the first items win in sys.path resolution.
    for p in reversed(paths):
        if p not in sys.path:
            sys.path.insert(0, p)


_configure_pycache_prefix()
_bootstrap_module_search_path()

import actions

try:
    import urdf_viewer
except Exception:  # pragma: no cover
    urdf_viewer = None  # type: ignore

try:
    import requests  # type: ignore
except Exception:  # pragma: no cover
    requests = None  # type: ignore


def _workspace_root_dir() -> str:
    # py_action/gui/app.py -> workspace root (ws)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))


def _resolve_existing_config_path(p: str) -> str:
    p = str(p)
    if os.path.isabs(p) and os.path.exists(p):
        return p
    candidates: list[str] = []
    # 1) cwd-based
    candidates.append(os.path.abspath(p))
    # 2) workspace-root-based
    wr = _workspace_root_dir()
    candidates.append(os.path.abspath(os.path.join(wr, p)))
    if not p.replace("\\", "/").startswith("py_action/"):
        candidates.append(os.path.abspath(os.path.join(wr, "py_action", p)))

    for cand in candidates:
        if os.path.exists(cand):
            return cand
    if getattr(sys, "frozen", False):
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            cand = os.path.join(str(meipass), p)
            if os.path.exists(cand):
                return os.path.abspath(cand)
    # Fall back to cwd-based absolute path for error messages
    return os.path.abspath(p)


def _try_read_yaml_as_dict(path: str) -> Optional[dict]:
    # Prefer PyYAML if available; fall back to a minimal text parser for the
    # few keys we care about so the GUI can still operate without pyyaml.
    try:
        import yaml  # type: ignore
    except Exception:
        yaml = None  # type: ignore

    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
    except Exception:
        return None

    if yaml is not None:
        try:
            data = yaml.safe_load(text) or {}
            return data if isinstance(data, dict) else None
        except Exception:
            pass

    return _try_read_overcurrent_minimal(text)


def _try_read_overcurrent_minimal(text: str) -> Optional[dict]:
    """Very small YAML-ish parser for overcurrent_protection.

    Only supports the subset used by this project.
    """
    try:
        lines = text.splitlines()
        out: dict = {}
        oc: dict = {}
        in_oc = False
        oc_indent: Optional[int] = None
        per_indent: Optional[int] = None
        in_per = False
        per_map: dict[int, float] = {}
        in_calib = False
        calib_indent: Optional[int] = None
        calib_map: dict[int, float] = {}

        for raw in lines:
            line = raw.rstrip("\n")
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue

            indent = len(line) - len(line.lstrip(" "))

            if stripped.startswith("overcurrent_protection:"):
                in_oc = True
                in_per = False
                oc_indent = indent
                per_indent = None
                continue

            if in_oc and oc_indent is not None and indent <= oc_indent and ":" in stripped:
                in_oc = False
                in_per = False

            if not in_oc:
                continue

            if stripped.startswith("per_servo_limit_ma:"):
                in_per = True
                per_indent = indent
                in_calib = False
                calib_indent = None
                continue

            if stripped.startswith("calibration_max_ma_by_id:"):
                in_calib = True
                calib_indent = indent
                in_per = False
                per_indent = None
                continue

            if in_per and per_indent is not None and indent <= per_indent and ":" in stripped:
                in_per = False

            if in_calib and calib_indent is not None and indent <= calib_indent and ":" in stripped:
                in_calib = False

            if in_per:
                if ":" not in stripped:
                    continue
                k, v = stripped.split(":", 1)
                try:
                    sid = int(str(k).strip())
                    ma = float(str(v).strip())
                    if 0 <= sid <= 253 and ma > 0:
                        per_map[int(sid)] = float(ma)
                except Exception:
                    continue
                continue

            if in_calib:
                if ":" not in stripped:
                    continue
                k, v = stripped.split(":", 1)
                try:
                    sid = int(str(k).strip())
                    ma = float(str(v).strip())
                    if 0 <= sid <= 253 and ma > 0:
                        calib_map[int(sid)] = float(ma)
                except Exception:
                    continue
                continue

            if ":" in stripped:
                k, v = stripped.split(":", 1)
                key = str(k).strip()
                val_s = str(v).strip()
                try:
                    if key in ("enabled", "latch"):
                        oc[key] = val_s.lower() in ("1", "true", "yes", "on")
                    elif key in ("default_limit_ma", "limit_ma", "reset_ratio", "calibration_margin_ma"):
                        oc[key] = float(val_s)
                except Exception:
                    pass

        if per_map:
            oc["per_servo_limit_ma"] = per_map
        if calib_map:
            oc["calibration_max_ma_by_id"] = calib_map
        if oc:
            out["overcurrent_protection"] = oc
        return out if out else {}
    except Exception:
        return None


def _try_write_yaml_dict(path: str, data: dict) -> bool:
    try:
        import yaml  # type: ignore
    except Exception:
        return False
    try:
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        return True
    except Exception:
        return False


def _update_overcurrent_per_servo_block(text: str, per_map: dict[int, int]) -> str:
    """Update/insert overcurrent_protection.per_servo_limit_ma in a YAML-ish text file."""
    lines = text.splitlines()
    if not lines:
        lines = []

    def _fmt_map(base_indent: int) -> list[str]:
        out_lines: list[str] = []
        out_lines.append(" " * base_indent + "per_servo_limit_ma:")
        for sid in sorted(per_map.keys()):
            out_lines.append(" " * (base_indent + 2) + f"{int(sid)}: {int(per_map[sid])}")
        return out_lines

    oc_start = None
    oc_indent = 0
    for i, raw in enumerate(lines):
        if raw.strip().startswith("overcurrent_protection:"):
            oc_start = i
            oc_indent = len(raw) - len(raw.lstrip(" "))
            break

    if oc_start is None:
        if lines and lines[-1].strip() != "":
            lines.append("")
        lines.append("overcurrent_protection:")
        lines.extend(_fmt_map(2))
        return "\n".join(lines) + "\n"

    oc_end = len(lines)
    for j in range(oc_start + 1, len(lines)):
        raw = lines[j]
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        ind = len(raw) - len(raw.lstrip(" "))
        if ind <= oc_indent and ":" in s:
            oc_end = j
            break

    per_line = None
    per_indent = oc_indent + 2
    for j in range(oc_start + 1, oc_end):
        if lines[j].strip().startswith("per_servo_limit_ma:"):
            per_line = j
            per_indent = len(lines[j]) - len(lines[j].lstrip(" "))
            break

    if per_line is None:
        lines[oc_end:oc_end] = _fmt_map(per_indent)
        return "\n".join(lines) + "\n"

    replace_start = per_line
    replace_end = per_line + 1
    for j in range(per_line + 1, oc_end):
        raw = lines[j]
        s = raw.strip()
        if not s or s.startswith("#"):
            replace_end = j + 1
            continue
        ind = len(raw) - len(raw.lstrip(" "))
        if ind <= per_indent and ":" in s:
            break
        replace_end = j + 1

    lines[replace_start:replace_end] = _fmt_map(per_indent)
    return "\n".join(lines) + "\n"


def _update_overcurrent_scalar(text: str, key: str, value: str) -> str:
    """Update/insert a scalar key under overcurrent_protection in a YAML-ish text file."""
    lines = text.splitlines()
    if not lines:
        lines = []

    # Find overcurrent_protection block
    oc_start = None
    oc_indent = 0
    for i, raw in enumerate(lines):
        if raw.strip().startswith("overcurrent_protection:"):
            oc_start = i
            oc_indent = len(raw) - len(raw.lstrip(" "))
            break

    if oc_start is None:
        if lines and lines[-1].strip() != "":
            lines.append("")
        lines.append("overcurrent_protection:")
        lines.append(" " * 2 + f"{key}: {value}")
        return "\n".join(lines) + "\n"

    # Determine overcurrent_protection block end
    oc_end = len(lines)
    for j in range(oc_start + 1, len(lines)):
        raw = lines[j]
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        ind = len(raw) - len(raw.lstrip(" "))
        if ind <= oc_indent and ":" in s:
            oc_end = j
            break

    # Replace if key exists
    key_line = None
    key_indent = oc_indent + 2
    for j in range(oc_start + 1, oc_end):
        s = lines[j].strip()
        if s.startswith(f"{key}:"):
            key_line = j
            key_indent = len(lines[j]) - len(lines[j].lstrip(" "))
            break

    new_line = " " * key_indent + f"{key}: {value}"
    if key_line is not None:
        lines[key_line] = new_line
        return "\n".join(lines) + "\n"

    # Insert before per_servo_limit_ma if present, otherwise at block end
    insert_at = oc_end
    for j in range(oc_start + 1, oc_end):
        if lines[j].strip().startswith("per_servo_limit_ma:"):
            insert_at = j
            break
    lines[insert_at:insert_at] = [new_line]
    return "\n".join(lines) + "\n"


def _update_overcurrent_int_map_block(text: str, map_key: str, id_to_value: dict[int, int]) -> str:
    """Update/insert overcurrent_protection.<map_key> as an int->int mapping."""
    lines = text.splitlines()
    if not lines:
        lines = []

    def _fmt_map(base_indent: int) -> list[str]:
        out_lines: list[str] = []
        out_lines.append(" " * base_indent + f"{map_key}:")
        for sid in sorted(id_to_value.keys()):
            out_lines.append(" " * (base_indent + 2) + f"{int(sid)}: {int(id_to_value[sid])}")
        return out_lines

    # Find overcurrent_protection block
    oc_start = None
    oc_indent = 0
    for i, raw in enumerate(lines):
        if raw.strip().startswith("overcurrent_protection:"):
            oc_start = i
            oc_indent = len(raw) - len(raw.lstrip(" "))
            break

    if oc_start is None:
        if lines and lines[-1].strip() != "":
            lines.append("")
        lines.append("overcurrent_protection:")
        lines.extend(_fmt_map(2))
        return "\n".join(lines) + "\n"

    # Determine block end
    oc_end = len(lines)
    for j in range(oc_start + 1, len(lines)):
        raw = lines[j]
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        ind = len(raw) - len(raw.lstrip(" "))
        if ind <= oc_indent and ":" in s:
            oc_end = j
            break

    # Find existing map within block
    map_line = None
    map_indent = oc_indent + 2
    for j in range(oc_start + 1, oc_end):
        if lines[j].strip().startswith(f"{map_key}:"):
            map_line = j
            map_indent = len(lines[j]) - len(lines[j].lstrip(" "))
            break

    if map_line is None:
        # Insert before per_servo_limit_ma if present (so it's near related config)
        insert_at = oc_end
        for j in range(oc_start + 1, oc_end):
            if lines[j].strip().startswith("per_servo_limit_ma:"):
                insert_at = j
                break
        lines[insert_at:insert_at] = _fmt_map(map_indent)
        return "\n".join(lines) + "\n"

    # Replace existing map block
    replace_start = map_line
    replace_end = map_line + 1
    for j in range(map_line + 1, oc_end):
        raw = lines[j]
        s = raw.strip()
        if not s or s.startswith("#"):
            replace_end = j + 1
            continue
        ind = len(raw) - len(raw.lstrip(" "))
        if ind <= map_indent and ":" in s:
            break
        replace_end = j + 1

    lines[replace_start:replace_end] = _fmt_map(map_indent)
    return "\n".join(lines) + "\n"


def _read_overcurrent_calibration_max_ma_by_id(config_path: str) -> dict[int, float]:
    try:
        cfg_path = _resolve_existing_config_path(str(config_path))
        data = _try_read_yaml_as_dict(cfg_path) or {}
    except Exception:
        data = {}

    oc = data.get("overcurrent_protection") if isinstance(data, dict) else None
    if not isinstance(oc, dict):
        return {}
    raw = oc.get("calibration_max_ma_by_id", None)
    out: dict[int, float] = {}
    if isinstance(raw, dict):
        for k, v in raw.items():
            try:
                sid = int(k)
                ma = float(v)
                if 0 <= sid <= 253 and ma > 0:
                    out[int(sid)] = float(ma)
            except Exception:
                continue
    return out


def _write_overcurrent_calibration_max_to_yaml(config_path: str, max_ma_by_id: dict[int, float]) -> str:
    cfg_path = _resolve_existing_config_path(str(config_path))
    if not os.path.exists(cfg_path):
        raise RuntimeError(f"配置文件不存在：{cfg_path}")

    out_map: dict[int, int] = {}
    for sid, v in (max_ma_by_id or {}).items():
        try:
            sid_i = int(sid)
            ma = float(v)
            if 0 <= sid_i <= 253 and ma > 0:
                out_map[int(sid_i)] = int(round(ma))
        except Exception:
            continue

    data = _try_read_yaml_as_dict(cfg_path)
    if isinstance(data, dict):
        try:
            oc_prev = data.get("overcurrent_protection")
            oc_new = dict(oc_prev) if isinstance(oc_prev, dict) else {}
            oc_new["calibration_max_ma_by_id"] = out_map
            new_data = dict(data)
            new_data["overcurrent_protection"] = oc_new
            if _try_write_yaml_dict(cfg_path, new_data):
                return os.path.abspath(cfg_path)
        except Exception:
            pass

    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            text = f.read()
    except Exception as e:
        raise RuntimeError(f"无法读取配置文件：{cfg_path}\n{e}")

    new_text = _update_overcurrent_int_map_block(text, "calibration_max_ma_by_id", out_map)
    try:
        with open(cfg_path, "w", encoding="utf-8") as f:
            f.write(new_text)
    except Exception as e:
        raise RuntimeError(f"无法写入配置文件：{cfg_path}\n{e}")

    return os.path.abspath(cfg_path)


def _read_overcurrent_calibration_margin_ma(config_path: str) -> float | None:
    try:
        cfg_path = _resolve_existing_config_path(str(config_path))
        data = _try_read_yaml_as_dict(cfg_path) or {}
    except Exception:
        data = {}

    oc = data.get("overcurrent_protection") if isinstance(data, dict) else None
    if not isinstance(oc, dict):
        return None
    try:
        v = oc.get("calibration_margin_ma", None)
        if v is None:
            return None
        return float(v)
    except Exception:
        return None


def _write_overcurrent_calibration_margin_to_yaml(config_path: str, margin_ma: float) -> str:
    cfg_path = _resolve_existing_config_path(str(config_path))
    if not os.path.exists(cfg_path):
        raise RuntimeError(f"配置文件不存在：{cfg_path}")

    try:
        margin_i = int(round(float(margin_ma)))
    except Exception:
        raise RuntimeError("calibration_margin_ma must be a number")
    if margin_i < 0:
        margin_i = 0

    data = _try_read_yaml_as_dict(cfg_path)
    if isinstance(data, dict):
        try:
            oc_prev = data.get("overcurrent_protection")
            oc_new = dict(oc_prev) if isinstance(oc_prev, dict) else {}
            oc_new["calibration_margin_ma"] = int(margin_i)
            new_data = dict(data)
            new_data["overcurrent_protection"] = oc_new
            if _try_write_yaml_dict(cfg_path, new_data):
                return os.path.abspath(cfg_path)
        except Exception:
            pass

    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            text = f.read()
    except Exception as e:
        raise RuntimeError(f"无法读取配置文件：{cfg_path}\n{e}")

    new_text = _update_overcurrent_scalar(text, "calibration_margin_ma", str(int(margin_i)))
    try:
        with open(cfg_path, "w", encoding="utf-8") as f:
            f.write(new_text)
    except Exception as e:
        raise RuntimeError(f"无法写入配置文件：{cfg_path}\n{e}")

    return os.path.abspath(cfg_path)


def _write_overcurrent_limits_to_yaml(config_path: str, per_servo_limit_ma: dict[int, float]) -> str:
    cfg_path = _resolve_existing_config_path(str(config_path))
    if not os.path.exists(cfg_path):
        raise RuntimeError(f"配置文件不存在：{cfg_path}")

    out_map: dict[int, int] = {}
    for sid, v in (per_servo_limit_ma or {}).items():
        try:
            sid_i = int(sid)
            ma = float(v)
            if 0 <= sid_i <= 253 and ma > 0:
                out_map[int(sid_i)] = int(round(ma))
        except Exception:
            continue

    data = _try_read_yaml_as_dict(cfg_path)
    if isinstance(data, dict):
        try:
            oc_prev = data.get("overcurrent_protection")
            oc_new = dict(oc_prev) if isinstance(oc_prev, dict) else {}
            oc_new["per_servo_limit_ma"] = out_map
            new_data = dict(data)
            new_data["overcurrent_protection"] = oc_new
            if _try_write_yaml_dict(cfg_path, new_data):
                return os.path.abspath(cfg_path)
        except Exception:
            pass

    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            text = f.read()
    except Exception as e:
        raise RuntimeError(f"无法读取配置文件：{cfg_path}\n{e}")

    new_text = _update_overcurrent_per_servo_block(text, out_map)
    try:
        with open(cfg_path, "w", encoding="utf-8") as f:
            f.write(new_text)
    except Exception as e:
        raise RuntimeError(f"无法写入配置文件：{cfg_path}\n{e}")

    return os.path.abspath(cfg_path)


def _write_runtime_config(base_config_path: str, serial_port: str) -> str:
    src = _resolve_existing_config_path(base_config_path)
    data = _try_read_yaml_as_dict(src) or {}

    if serial_port:
        data["port"] = serial_port
    if "baudrate" not in data:
        data["baudrate"] = 1_000_000

    out_dir = _ocra_temp_dir()
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"runtime_config_{serial_port or 'AUTO'}.yaml")

    try:
        import yaml  # type: ignore

        with open(out_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    except Exception:
        # Worst-case fallback: still write a minimal config.
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(f"port: {serial_port}\nbaudrate: 1000000\n")
    return os.path.abspath(out_path)


def _read_overcurrent_config(config_path: str) -> tuple[bool, float | None, dict[int, float], bool, float]:
    """Read per-servo overcurrent limits from YAML.

    Returns: (enabled, default_limit_ma, per_servo_limit_ma, latch, reset_ratio)
    """

    enabled = False
    default_limit_ma: float | None = None
    per_servo: dict[int, float] = {}
    latch = True
    reset_ratio = 0.80

    try:
        cfg_path = _resolve_existing_config_path(str(config_path))
        data = _try_read_yaml_as_dict(cfg_path) or {}
    except Exception:
        data = {}

    oc = data.get("overcurrent_protection") if isinstance(data, dict) else None
    if isinstance(oc, dict):
        try:
            enabled = bool(oc.get("enabled", False))
        except Exception:
            enabled = False
        try:
            v = oc.get("default_limit_ma", None)
            if v is None:
                v = oc.get("limit_ma", None)
            if v is not None:
                default_limit_ma = float(v)
        except Exception:
            default_limit_ma = None
        try:
            latch = bool(oc.get("latch", True))
        except Exception:
            latch = True
        try:
            reset_ratio = float(oc.get("reset_ratio", 0.80))
        except Exception:
            reset_ratio = 0.80

        raw = oc.get("per_servo_limit_ma", None)
        if isinstance(raw, dict):
            for k, v in raw.items():
                try:
                    sid = int(k)
                    ma = float(v)
                    if 0 <= sid <= 253 and ma > 0:
                        per_servo[int(sid)] = float(ma)
                except Exception:
                    continue

    return enabled, default_limit_ma, per_servo, latch, reset_ratio


def _maybe_run_worker_mode() -> None:
    """When packaged as a single EXE, `sys.executable` points to the EXE itself.

    The legacy GUI starts API/camera via subprocess. In a frozen build that would
    otherwise re-launch the GUI. We support worker flags so the child process can
    run API/camera logic without creating any Tk UI.
    """

    argv = list(sys.argv[1:])

    _log_dir = _ocra_temp_dir()
    try:
        os.makedirs(_log_dir, exist_ok=True)
    except Exception:
        pass
    _log_path = os.path.join(_log_dir, "worker.log")

    def _wlog(msg: str) -> None:
        try:
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(_log_path, "a", encoding="utf-8") as f:
                f.write(f"[{ts}] {msg}\n")
        except Exception:
            pass
    if "--run-api" in argv:
        _wlog(f"enter --run-api argv={argv}")
        host = "127.0.0.1"
        port = 8001
        if "--host" in argv:
            try:
                host = argv[argv.index("--host") + 1]
            except Exception:
                pass
        if "--port" in argv:
            try:
                port = int(argv[argv.index("--port") + 1])
            except Exception:
                pass

        def _has_route(app_obj, path: str) -> bool:
            try:
                for r in getattr(getattr(app_obj, "router", None), "routes", []) or []:
                    if getattr(r, "path", None) == path:
                        return True
            except Exception:
                return False
            return False

        def _import_api_app():
            last_err: Optional[Exception] = None
            for mod_name in ("server",):
                try:
                    mod = importlib.import_module(mod_name)
                    app_obj = getattr(mod, "app", None)
                    if app_obj is None:
                        continue
                    if _has_route(app_obj, "/config"):
                        return app_obj
                except Exception as e:
                    last_err = e
                    continue
            raise RuntimeError(f"Failed to import correct FastAPI app (last error: {last_err})")

        try:
            try:
                import uvicorn  # type: ignore
                _wlog("import uvicorn OK")
            except Exception as e:
                raise RuntimeError(f"uvicorn not available: {e}")

            api_app = _import_api_app()
            _wlog(f"import api app OK title={getattr(api_app, 'title', None)}")
            _wlog(f"starting uvicorn host={host} port={port}")
            uvicorn.run(
                api_app,
                host=host,
                port=port,
                log_level="info",
                access_log=True,
                use_colors=False,
            )
            _wlog("uvicorn.run returned")
        except Exception as e:
            print(f"[worker api] failed: {e}")
            _wlog(f"[worker api] failed: {e}\n{traceback.format_exc()}")
            raise
        finally:
            raise SystemExit(0)

    if "--run-camera3d" in argv:
        _wlog(f"enter camera worker argv={argv}")
        base = "http://127.0.0.1:8001"
        cfg = "py_action/config_orca.yaml"
        hand = "left"
        hz = "10"
        ema = "0.75"
        cam = "0"
        if "--base" in argv:
            try:
                base = argv[argv.index("--base") + 1]
            except Exception:
                pass
        if "--config" in argv:
            try:
                cfg = argv[argv.index("--config") + 1]
            except Exception:
                pass
        if "--hand" in argv:
            try:
                hand = argv[argv.index("--hand") + 1]
            except Exception:
                pass
        if "--hz" in argv:
            try:
                hz = argv[argv.index("--hz") + 1]
            except Exception:
                pass
        if "--ema" in argv:
            try:
                ema = argv[argv.index("--ema") + 1]
            except Exception:
                pass
        if "--camera" in argv:
            try:
                cam = argv[argv.index("--camera") + 1]
            except Exception:
                pass

        cfg = _resolve_existing_config_path(cfg)
        # serial port override (optional): if not provided, keep cfg as-is
        if "--serial-port" in argv:
            try:
                sp = argv[argv.index("--serial-port") + 1].strip()
            except Exception:
                sp = ""
            if sp:
                cfg = _write_runtime_config(cfg, sp)
                _wlog(f"camera runtime config written: {cfg}")

        try:
            import camera3d_hand_to_api as cam_mod  # type: ignore

            sys.argv = [
                "camera",
                "--base",
                base,
                "--config",
                cfg,
                "--hand",
                hand,
                "--hz",
                hz,
                "--ema",
                ema,
                "--camera",
                cam,
            ]
            cam_mod.main()
        except SystemExit:
            raise
        except Exception as e:
            # If this is an HTTP error, try to print response body to help debugging (e.g. config file not found)
            try:
                import requests  # type: ignore

                if isinstance(e, requests.HTTPError) and e.response is not None:
                    print(f"[worker camera] http status={e.response.status_code} body={e.response.text}")
            except Exception:
                pass
            print(f"[worker camera] failed: {e}")
            _wlog(f"[worker camera] failed: {e}\n{traceback.format_exc()}")
            raise
        finally:
            raise SystemExit(0)


_maybe_run_worker_mode()

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

# ======================
# GUI 初始化
# ======================
root = tk.Tk()
root.title("OrcaHand Plus 调试上位机")
root.geometry("1080x920")
root.minsize(980, 820)
root.resizable(True, True)

BG = "#050805"
PANEL = "#101510"
PANEL_2 = "#151C16"
BORDER = "#263328"
FG = "#EAF4EA"
MUTED = "#9BAA9C"
ACCENT = "#76B900"
ACCENT_2 = "#2EE66B"
BTN = "#2E7D22"
BTN_DANGER = "#D93434"
BTN_NEUTRAL = "#263328"
BTN_WARN = "#B7791F"
ENTRY_BG = "#071007"
DISABLED_BG = "#172017"

root.configure(bg=BG)

# GUI 关闭信号：用于让后台线程退出，避免窗口关闭后进程残留
closing_event = threading.Event()

# 轮询挂起：用于动作7标定时减少总线竞争。
# 注意：如果这个事件未定义/线程崩溃，会导致位置(0-4095)无法回读。
polling_suspend_event = threading.Event()
polling_suspend_ts = 0.0


def _suspend_polling(reason: str = "") -> None:
    global polling_suspend_ts
    try:
        polling_suspend_ts = float(time.time())
    except Exception:
        polling_suspend_ts = 0.0
    try:
        polling_suspend_event.set()
    except Exception:
        pass


def _resume_polling() -> None:
    try:
        polling_suspend_event.clear()
    except Exception:
        pass

font_btn = ("微软雅黑", 10, "bold")
font_title = ("微软雅黑", 18, "bold")
font_status = ("微软雅黑", 10, "bold")


def _configure_panel(widget: tk.Widget) -> None:
    try:
        bg = str(widget.cget("bg") or PANEL)
        widget.configure(bg=bg, highlightbackground=BORDER, highlightcolor=ACCENT, highlightthickness=1, bd=0)
    except Exception:
        pass


def _style_button(widget: tk.Widget, *, variant: str = "primary") -> None:
    palette = {
        "primary": (BTN, "white", ACCENT),
        "danger": (BTN_DANGER, "white", "#FF7A7A"),
        "neutral": (BTN_NEUTRAL, FG, BORDER),
        "warn": (BTN_WARN, "white", "#E0A12D"),
    }
    bg, fg, active_bg = palette.get(variant, palette["primary"])
    try:
        widget.configure(
            bg=bg,
            fg=fg,
            activebackground=active_bg,
            activeforeground="white",
            relief="flat",
            bd=0,
            highlightthickness=0,
            padx=8,
            pady=4,
            cursor="hand2",
        )
    except Exception:
        pass


def _style_widget_tree(widget: tk.Widget) -> None:
    """Best-effort dark/NVIDIA-like skin without changing widget behavior."""
    try:
        cls = widget.winfo_class()
    except Exception:
        cls = ""

    try:
        if cls in ("Frame", "Labelframe"):
            current_bg = ""
            try:
                current_bg = str(widget.cget("bg"))
            except Exception:
                pass
            if current_bg.lower() in (PANEL.lower(), PANEL_2.lower(), ENTRY_BG.lower(), "#071007"):
                _configure_panel(widget)
            else:
                widget.configure(bg=BG)
        elif cls == "Label":
            bg = PANEL if str(widget.master.winfo_class()) in ("Frame", "Labelframe") else BG
            try:
                master_bg = widget.master.cget("bg")
                if str(master_bg).lower() in (PANEL.lower(), PANEL_2.lower(), ENTRY_BG.lower()):
                    bg = str(master_bg)
            except Exception:
                pass
            widget.configure(bg=bg, fg=FG)
            text = str(widget.cget("text") or "")
            if "提示" in text or "注意" in text or "模型会" in text or "波特率" in text:
                widget.configure(fg=MUTED)
        elif cls == "Button":
            text = str(widget.cget("text") or "")
            if any(key in text for key in ("急停", "停止", "失败")):
                _style_button(widget, variant="danger")
            elif any(key in text for key in ("关闭", "断开", "返回")):
                _style_button(widget, variant="neutral")
            elif any(key in text for key in ("暂停", "加值")):
                _style_button(widget, variant="warn")
            else:
                _style_button(widget, variant="primary")
        elif cls == "Checkbutton":
            widget.configure(
                bg=PANEL,
                fg=FG,
                activebackground=PANEL,
                activeforeground=ACCENT_2,
                selectcolor=ENTRY_BG,
                highlightthickness=0,
            )
        elif cls in ("Entry", "Spinbox"):
            widget.configure(
                bg=ENTRY_BG,
                fg=FG,
                insertbackground=ACCENT_2,
                relief="flat",
                highlightbackground=BORDER,
                highlightcolor=ACCENT,
                highlightthickness=1,
            )
        elif cls == "Text":
            widget.configure(
                bg="#050A05",
                fg="#BFEFBD",
                insertbackground=ACCENT_2,
                relief="flat",
                highlightbackground=BORDER,
                highlightthickness=1,
            )
        elif cls == "Canvas":
            try:
                current_bg = str(widget.cget("bg") or PANEL)
            except Exception:
                current_bg = PANEL
            canvas_bg = current_bg if current_bg.lower() in (BG.lower(), ENTRY_BG.lower(), "#071007") else PANEL
            widget.configure(bg=canvas_bg, highlightbackground=BORDER, highlightthickness=1)
        elif cls == "Scale":
            widget.configure(
                bg=PANEL,
                fg=FG,
                troughcolor="#1C2A1D",
                activebackground=ACCENT,
                highlightthickness=0,
            )
        elif cls == "Menubutton":
            widget.configure(bg=ENTRY_BG, fg=FG, activebackground=BTN_NEUTRAL, activeforeground=FG, relief="flat")
    except Exception:
        pass

    for child in widget.winfo_children():
        _style_widget_tree(child)


def _spawn_cwd() -> str:
    # In frozen builds, __file__ may be unavailable or point to extraction dir.
    # Using current working directory is generally the least surprising.
    if getattr(sys, "frozen", False):
        return os.getcwd()
    try:
        # Keep child processes rooted at the workspace so paths like
        # "py_action/config_orca.yaml" resolve consistently.
        return _workspace_root_dir()
    except Exception:
        return os.getcwd()


_WORKSPACE_ROOT = _spawn_cwd()


def _is_proc_running(p: subprocess.Popen | None) -> bool:
    try:
        return p is not None and p.poll() is None
    except Exception:
        return False


_log_q: "queue.Queue[str]" = queue.Queue()


def _enqueue_log(prefix: str, line: str) -> None:
    try:
        _log_q.put_nowait(f"[{prefix}] {line}")
    except Exception:
        pass


def _start_reader_thread(prefix: str, p: subprocess.Popen) -> None:
    def _reader():
        try:
            if p.stdout is None:
                return
            for raw in p.stdout:
                s = str(raw).rstrip("\n")
                if s:
                    _enqueue_log(prefix, s)
        except Exception:
            pass

    threading.Thread(target=_reader, daemon=True).start()


def _spawn_process(prefix: str, argv: list[str], extra_env: dict[str, str] | None = None) -> subprocess.Popen:
    creationflags = 0
    if os.name == "nt":
        # allow CTRL_BREAK_EVENT if possible
        creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)

    env = None
    if extra_env:
        try:
            env = os.environ.copy()
            for k, v in extra_env.items():
                if v is None:
                    continue
                env[str(k)] = str(v)
        except Exception:
            env = None

    p = subprocess.Popen(
        argv,
        cwd=_WORKSPACE_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        creationflags=creationflags,
        env=env,
    )
    _enqueue_log(prefix, f"started pid={getattr(p, 'pid', '?')} argv={' '.join(argv)}")
    _start_reader_thread(prefix, p)
    return p


def _stop_process(prefix: str, p: subprocess.Popen | None) -> None:
    if p is None:
        return
    if not _is_proc_running(p):
        return
    proc = p
    try:
        if os.name == "nt":
            try:
                proc.send_signal(signal.CTRL_BREAK_EVENT)
                time.sleep(0.2)
            except Exception:
                pass
        proc.terminate()
    except Exception:
        pass

    def _wait_kill():
        try:
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        _enqueue_log(prefix, "stopped")

    threading.Thread(target=_wait_kill, daemon=True).start()


def list_serial_ports():
    if list_ports is None:
        return []
    return [p.device for p in list_ports.comports()]

# ======================
# 状态栏
# ======================
status = tk.Label(
    root,
    text="🟢 空闲",
    font=font_status,
    bg=BG,
    fg=ACCENT_2,
    padx=12,
    pady=4,
)
status.pack(pady=3)


def safe_call(fn):
    try:
        return fn()
    except Exception:
        return None


def safe_after(fn):
    try:
        root.after(0, fn)
    except Exception:
        pass

def set_status(t):
    if "空闲" in t or "完成" in t:
        color = ACCENT_2
    elif "执行" in t:
        color = "#6BE7FF"
    elif "紧急" in t:
        color = "#FFD166"
    else:
        color = "#FF6B6B"
    status.config(text=t, fg=color)


def set_status_safe(t):
    """Tkinter 只能在主线程更新，动作线程里要用 after 回到主线程。"""
    safe_after(lambda: set_status(t))


# 临时状态提示：显示一会儿后自动回到'空闲'（避免一直占着状态栏）
_status_temp_token = 0


def set_status_temp_safe(t: str, *, ttl_ms: int = 2500):
    global _status_temp_token
    try:
        _status_temp_token += 1
        token = int(_status_temp_token)
    except Exception:
        token = 0

    def _apply():
        try:
            set_status(str(t))
        except Exception:
            return

        def _maybe_restore():
            try:
                if int(_status_temp_token) != int(token):
                    return
            except Exception:
                pass
            try:
                if bool(getattr(actions, "is_action_running", lambda: False)()):
                    return
            except Exception:
                pass
            try:
                set_status("🟢 空闲")
            except Exception:
                pass

        try:
            root.after(max(500, int(ttl_ms)), _maybe_restore)
        except Exception:
            pass

    safe_after(_apply)


def _on_app_close():
    # 先通知后台线程退出
    closing_event.set()

    # best-effort: stop child processes started from the toolbox
    def stop_children():
        try:
            _stop_process("CAM3D", globals().get("cam3d_proc"))  # type: ignore[arg-type]
            _stop_process("API", globals().get("api_proc"))  # type: ignore[arg-type]
        except Exception:
            pass

    threading.Thread(target=stop_children, daemon=True).start()

    # 清理串口/急停不要阻塞 UI 线程，避免关窗卡死
    def cleanup():
        safe_call(actions.emergency_stop)
        safe_call(actions.disconnect)

    threading.Thread(target=cleanup, daemon=True).start()

    safe_call(_stop_3d_viewer)

    safe_call(root.quit)
    safe_call(root.destroy)


root.protocol("WM_DELETE_WINDOW", _on_app_close)

container = tk.Frame(root, bg=BG)
container.pack(fill="both", expand=True)

config_frame = tk.Frame(container, bg=BG)
panel_frame = tk.Frame(container, bg=BG)

for f in (config_frame, panel_frame):
    f.place(relx=0, rely=0, relwidth=1, relheight=1)


def show_frame(frame: tk.Frame):
    frame.tkraise()


# ======================
# 配置页：首页控制台
# ======================
home_frame = tk.Frame(config_frame, bg=BG)
home_frame.pack(fill="both", expand=True, padx=22, pady=(8, 16))

hero = tk.Frame(home_frame, bg=BG)
hero.pack(fill="x", pady=(0, 14))

brand = tk.Frame(hero, bg=BG)
brand.pack(side="left", fill="x", expand=True)

tk.Label(
    brand,
    text="ORCAHAND PLUS",
    font=("Segoe UI", 10, "bold"),
    bg=BG,
    fg=ACCENT,
).pack(anchor="w")

tk.Label(
    brand,
    text="灵巧手调试上位机",
    font=("微软雅黑", 24, "bold"),
    bg=BG,
    fg=FG,
).pack(anchor="w")

tk.Label(
    brand,
    text="FEETECH STS3215 CONTROL CONSOLE",
    font=("Consolas", 10),
    bg=BG,
    fg=MUTED,
).pack(anchor="w", pady=(3, 0))

hero_meter = tk.Canvas(hero, width=220, height=54, bg=BG, highlightthickness=0)
hero_meter.pack(side="right", padx=(20, 0))
hero_meter.create_line(8, 40, 212, 40, fill="#1D3B20", width=2)
for _x in (22, 58, 94, 130, 166, 202):
    hero_meter.create_line(_x, 35, _x + 15, 20, fill="#263328", width=2)
hero_meter.create_oval(166, 12, 194, 40, outline=ACCENT, width=2)
hero_meter.create_oval(173, 19, 187, 33, fill=ACCENT_2, outline="")
hero_meter.create_text(110, 50, text="READY BUS", fill=MUTED, font=("Consolas", 8))

dashboard = tk.Frame(home_frame, bg=BG)
dashboard.pack(fill="x")
dashboard.grid_columnconfigure(0, weight=1, uniform="home_card")
dashboard.grid_columnconfigure(1, weight=1, uniform="home_card")

cfg_panel = tk.Frame(dashboard, bg=PANEL)
cfg_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

serial_head = tk.Frame(cfg_panel, bg=PANEL)
serial_head.pack(fill="x", padx=18, pady=(16, 10))

tk.Label(
    serial_head,
    text="串口与调试",
    font=("微软雅黑", 15, "bold"),
    bg=PANEL,
    fg=FG,
).pack(side="left")

tk.Label(
    serial_head,
    text="1,000,000 BAUD",
    font=("Consolas", 9, "bold"),
    bg="#152416",
    fg=ACCENT_2,
    padx=10,
    pady=4,
).pack(side="right")

serial_body = tk.Frame(cfg_panel, bg=PANEL)
serial_body.pack(fill="x", padx=18, pady=(2, 12))
serial_body.grid_columnconfigure(1, weight=1)

tk.Label(serial_body, text="串口", bg=PANEL, fg=MUTED, font=("微软雅黑", 10)).grid(row=0, column=0, sticky="w", pady=8)

available_ports = list_serial_ports()
port_var = tk.StringVar(value=available_ports[0] if available_ports else "")
# tk.OptionMenu(master, variable, value, *values) 至少需要一个 value。
# 另一台电脑若未检测到串口，available_ports 为空会导致启动时直接异常。
if available_ports:
    port_menu = tk.OptionMenu(serial_body, port_var, available_ports[0], *available_ports[1:])
else:
    port_menu = tk.OptionMenu(serial_body, port_var, "")
    port_menu.config(state="disabled")
port_menu.config(width=14)
port_menu.grid(row=0, column=1, sticky="we", padx=(10, 8), pady=8)


def refresh_ports():
    ports = list_serial_ports()
    menu = port_menu["menu"]
    menu.delete(0, "end")
    if ports:
        for p in ports:
            menu.add_command(label=p, command=lambda v=p: port_var.set(v))
        if port_var.get() not in ports:
            port_var.set(ports[0])
        port_menu.config(state="normal")
    else:
        # menu 不能为空（否则有些 Tk 版本/打包环境行为不一致）
        menu.add_command(label="", command=lambda: port_var.set(""))
        port_var.set("")
        port_menu.config(state="disabled")


tk.Button(
    serial_body,
    text="刷新",
    font=("微软雅黑", 10),
    bg="#EEEEEE",
    relief="flat",
    command=refresh_ports,
).grid(row=0, column=2, sticky="e", padx=(0, 6), pady=8)


def _test_serial_port():
    port = port_var.get().strip()
    if not port:
        messagebox.showwarning("提示", "未选择串口。")
        return
    try:
        import serial  # type: ignore
    except Exception as e:
        messagebox.showerror("缺少依赖", f"pyserial 未安装：\n\n{e}")
        return

    try:
        s = serial.Serial(port, baudrate=1_000_000, timeout=0.2)
        s.close()
        messagebox.showinfo("串口测试", f"打开 {port} 成功。")
    except Exception as e:
        messagebox.showerror(
            "串口测试失败",
            (
                f"无法打开/配置 {port}（1,000,000 波特）。\n\n"
                f"常见原因：端口被占用、设备未上电/未插好、驱动异常。\n\n"
                f"原始错误：{e}"
            ),
        )


tk.Button(
    serial_body,
    text="测试",
    font=("微软雅黑", 10),
    bg="#EEEEEE",
    relief="flat",
    command=_test_serial_port,
).grid(row=0, column=3, sticky="e", pady=8)

debug_var = tk.BooleanVar(value=False)

# 当从控制面板返回串口配置时，会触发'急停回零 + 断开串口'。
# 该过程是异步的，若用户立刻又尝试连接，容易出现竞态（总线仍在回零/释放中）。
_debug_cleanup_in_progress = False


def set_debug_mode(enabled: bool):
    global _debug_cleanup_in_progress
    if enabled:
        if bool(_debug_cleanup_in_progress):
            debug_var.set(False)
            messagebox.showwarning("请稍候", "正在急停回零并断开串口，请稍等 1~3 秒后再重新连接。")
            return
        # 串口互斥：如果 API server 在跑，它也会占用串口，GUI 这边不要再连。
        if _is_proc_running(globals().get("api_proc")):  # type: ignore[arg-type]
            debug_var.set(False)
            messagebox.showwarning(
                "串口互斥",
                "检测到 API 服务正在运行（它会占用串口）。\n\n"
                "请先在首页工具箱里停止 API 服务，再进入调试面板连接串口。",
            )
            return
        port = port_var.get().strip()
        if not port:
            debug_var.set(False)
            messagebox.showwarning("提示", "未检测到串口，请先插入设备并点击'刷新'。")
            return
        try:
            actions.connect(port)
        except Exception as e:
            debug_var.set(False)
            set_status("🔴 连接失败")
            messagebox.showerror("连接失败", f"无法打开串口 {port}\n\n{e}")
            return

        # 连接成功后自动回零一次
        try:
            set_status("🟡 连接成功，正在回零")
            actions.action_all_zero()
        except Exception as e:
            # 回零失败不阻止进入面板，但要给出提示
            set_status("🟠 回零失败")
            messagebox.showwarning("回零失败", f"已连接 {port}，但自动回零失败：\n\n{e}")

        set_status(f"🟢 已连接 {port}")
        show_frame(panel_frame)

        # 进入调试面板后默认开启过流保护（若有阈值配置）。
        try:
            if "oc_enable_var" in globals() and globals().get("oc_enable_var") is not None:
                globals()["oc_enable_var"].set(True)
            safe_after(_apply_overcurrent_params)
        except Exception:
            pass
    else:
        _debug_cleanup_in_progress = True

        # 不在 UI 线程里阻塞回零/断开，避免卡顿；但要保证断开发生在回零完成后。
        def _cleanup_then_show():
            global _debug_cleanup_in_progress
            try:
                try:
                    actions.emergency_stop(set_status_safe, wait=True)
                except Exception:
                    pass
                try:
                    actions.disconnect()
                except Exception:
                    pass
            finally:
                try:
                    # 回到配置页
                    safe_after(lambda: (set_status("🟢 空闲"), show_frame(config_frame)))
                except Exception:
                    pass
                try:
                    _debug_cleanup_in_progress = False
                except Exception:
                    pass

        threading.Thread(target=_cleanup_then_show, daemon=True).start()

        # 立即切回配置页（清理在后台进行）
        set_status("🟡 正在断开")
        show_frame(config_frame)


debug_box = tk.Frame(cfg_panel, bg=PANEL_2)
debug_box.pack(fill="x", padx=18, pady=(0, 18))

tk.Checkbutton(
    debug_box,
    text="连接并进入控制面板",
    variable=debug_var,
    onvalue=True,
    offvalue=False,
    bg=PANEL_2,
    command=lambda: set_debug_mode(debug_var.get()),
    font=("微软雅黑", 12, "bold"),
).pack(anchor="w", padx=14, pady=(12, 3))


tk.Label(
    debug_box,
    text="关闭连接时会急停回零并释放串口",
    bg=PANEL_2,
    fg=MUTED,
    font=("微软雅黑", 9),
).pack(anchor="w", padx=38, pady=(0, 12))


# ======================
# 启动页工具箱：一站式启动 API / 相机脚本
# ======================
tool_panel = tk.Frame(dashboard, bg=PANEL)
tool_panel.grid(row=0, column=1, sticky="nsew", padx=(10, 0))

tool_head = tk.Frame(tool_panel, bg=PANEL)
tool_head.pack(fill="x", padx=18, pady=(16, 10))

tk.Label(
    tool_head,
    text="API 与相机跟随",
    font=("微软雅黑", 15, "bold"),
    bg=PANEL,
    fg=FG,
).pack(side="left")

tk.Label(
    tool_head,
    text="SERIAL MUTEX",
    font=("Consolas", 9, "bold"),
    bg="#241B10",
    fg="#FFD166",
    padx=10,
    pady=4,
).pack(side="right")

row_cfg = tk.Frame(tool_panel, bg=PANEL)
row_cfg.pack(fill="x", padx=18, pady=(2, 10))

tk.Label(row_cfg, text="API base", bg=PANEL, fg=MUTED).grid(row=0, column=0, sticky="w", pady=6)
base_var = tk.StringVar(value="http://127.0.0.1:8001")
tk.Entry(row_cfg, textvariable=base_var, width=34).grid(row=0, column=1, sticky="we", padx=(10, 12), pady=6)

tk.Label(row_cfg, text="Port", bg=PANEL, fg=MUTED).grid(row=0, column=2, sticky="e", pady=6)
port_api_var = tk.StringVar(value="8001")
tk.Entry(row_cfg, textvariable=port_api_var, width=8).grid(row=0, column=3, sticky="e", pady=6)

tk.Label(row_cfg, text="Config", bg=PANEL, fg=MUTED).grid(row=1, column=0, sticky="w", pady=6)
config_path_var = tk.StringVar(value="py_action/config_orca.yaml")
tk.Entry(row_cfg, textvariable=config_path_var, width=34).grid(row=1, column=1, columnspan=3, sticky="we", padx=(10, 0), pady=6)

# 动作7：电流校准的"加值"(mA)，可落盘到 YAML（overcurrent_protection.calibration_margin_ma）
calib_margin_var = tk.StringVar(value="10")

# 当我们从 YAML/程序逻辑里同步 calib_margin_var 时，避免触发 trace_add 的'自动保存'回环。
_suppress_action7_margin_autosave = False

# 启动时先从 YAML 带出一次（避免 UI 默认值与文件不一致造成'没更新'的错觉）
try:
    _m0 = _read_overcurrent_calibration_margin_ma(config_path_var.get())
    if _m0 is not None:
        try:
            _suppress_action7_margin_autosave = True
            calib_margin_var.set(str(int(round(float(_m0)))))
        finally:
            _suppress_action7_margin_autosave = False
except Exception:
    pass


# ---- Over-current config (from YAML) ----
_oc_cfg_enabled, _oc_cfg_default_ma, _oc_cfg_by_id, _oc_cfg_latch, _oc_cfg_reset_ratio = _read_overcurrent_config(
    config_path_var.get()
)


def _effective_oc_limit_ma_for_sid(sid: int, *, default_limit_ma: float | None) -> float | None:
    v = _oc_cfg_by_id.get(int(sid))
    if v is not None:
        return float(v)
    if default_limit_ma is None:
        return None
    return float(default_limit_ma)


def _refresh_oc_limit_labels() -> None:
    # Called after UI created; safe no-op before.
    try:
        labels = globals().get("servo_limit_labels")
        if not isinstance(labels, dict):
            return
        # Determine runtime default limit (from UI) if available.
        try:
            default_v = globals().get("oc_default_ma_var")
            default_ma = float(int(default_v.get())) if default_v is not None else None
            if default_ma is not None and default_ma <= 0:
                default_ma = None
        except Exception:
            default_ma = None

        for sid, lbl in labels.items():
            lim = _effective_oc_limit_ma_for_sid(int(sid), default_limit_ma=default_ma)
            txt = "限: --" if lim is None else f"限: {float(lim):.0f}"
            try:
                lbl.config(text=txt)
            except Exception:
                pass
    except Exception:
        pass


def _reload_oc_config_from_yaml() -> None:
    global _oc_cfg_enabled, _oc_cfg_default_ma, _oc_cfg_by_id, _oc_cfg_latch, _oc_cfg_reset_ratio
    global _suppress_action7_margin_autosave
    _oc_cfg_enabled, _oc_cfg_default_ma, _oc_cfg_by_id, _oc_cfg_latch, _oc_cfg_reset_ratio = _read_overcurrent_config(
        config_path_var.get()
    )

    # Sync action-7 calibration margin from YAML when present.
    try:
        m = _read_overcurrent_calibration_margin_ma(config_path_var.get())
        if m is not None:
            try:
                try:
                    _suppress_action7_margin_autosave = True
                    calib_margin_var.set(str(int(round(float(m)))))
                finally:
                    _suppress_action7_margin_autosave = False
            except Exception:
                pass
    except Exception:
        pass
    # Sync UI defaults when possible (do not override user choice if already set).
    try:
        if "oc_enable_var" in globals() and globals().get("oc_enable_var") is not None:
            try:
                globals()["oc_enable_var"].set(bool(_oc_cfg_enabled))
            except Exception:
                pass
        if "oc_default_ma_var" in globals() and globals().get("oc_default_ma_var") is not None:
            try:
                cur = int(globals()["oc_default_ma_var"].get())
            except Exception:
                cur = 0
            if cur <= 0 and _oc_cfg_default_ma is not None:
                try:
                    globals()["oc_default_ma_var"].set(int(round(float(_oc_cfg_default_ma))))
                except Exception:
                    pass
    except Exception:
        pass

    safe_after(_refresh_oc_limit_labels)


def _on_config_path_change(*_):
    # User might edit path gradually; just best-effort reload.
    _reload_oc_config_from_yaml()


try:
    config_path_var.trace_add("write", _on_config_path_change)
except Exception:
    pass

tk.Label(row_cfg, text="Camera", bg=PANEL, fg=MUTED).grid(row=2, column=0, sticky="w", pady=6)
camera_index_var = tk.StringVar(value="0")

cam_row = tk.Frame(row_cfg, bg=PANEL)
cam_row.grid(row=2, column=1, sticky="w", padx=(10, 0), pady=6)
tk.Entry(cam_row, textvariable=camera_index_var, width=10).pack(side="left")


def _scan_cameras():
    """Probe camera indices and show available ones.

    We can't reliably get device *names* without extra deps, but index probing is
    sufficient for most Windows setups.
    """

    try:
        import cv2  # type: ignore
    except Exception as e:
        messagebox.showerror(
            "缺少依赖",
            "未安装 OpenCV（cv2），无法扫描相机。\n\n"
            "请先安装依赖：pip install -r .\\py_action\\requirements.txt\n\n"
            f"原始错误：{e}",
        )
        return

    # Keep UI responsive
    def _worker():
        max_idx = 8
        found: list[int] = []
        for idx in range(max_idx + 1):
            cap = None
            try:
                # On Windows, DirectShow is usually faster/less flaky.
                backend = getattr(cv2, "CAP_DSHOW", 0)
                cap = cv2.VideoCapture(idx, backend)
                if not cap or not bool(cap.isOpened()):
                    try:
                        if cap:
                            cap.release()
                    except Exception:
                        pass
                    cap = cv2.VideoCapture(idx)

                if cap and bool(cap.isOpened()):
                    ok, _frame = cap.read()
                    if ok:
                        found.append(idx)
            except Exception:
                pass
            finally:
                try:
                    if cap:
                        cap.release()
                except Exception:
                    pass

        def _show():
            if found:
                try:
                    camera_index_var.set(str(found[0]))
                except Exception:
                    pass
                messagebox.showinfo(
                    "相机扫描结果",
                    "检测到可用摄像头索引：\n\n"
                    + ", ".join(str(i) for i in found)
                    + "\n\n已自动选择第一个可用索引。",
                )
            else:
                messagebox.showwarning(
                    "相机扫描结果",
                    "未检测到可用摄像头索引（0~8）。\n\n"
                    "可能原因：\n"
                    "- 摄像头被其它程序占用（微信/QQ/浏览器/旧的相机进程）\n"
                    "- 权限未给（系统相机权限）\n"
                    "- 驱动异常/设备未连接",
                )

        safe_after(_show)

    threading.Thread(target=_worker, daemon=True).start()


tk.Button(
    cam_row,
    text="扫描",
    font=("微软雅黑", 10),
    bg="#EEEEEE",
    relief="flat",
    command=_scan_cameras,
).pack(side="left", padx=6)

row_cfg.grid_columnconfigure(1, weight=1)


api_proc: subprocess.Popen | None = None
cam3d_proc: subprocess.Popen | None = None

_api_version_cache: dict | None = None
_api_version_last_error: str = ""


def _refresh_tool_status():
    parts = []
    api_state = "RUN" if _is_proc_running(api_proc) else "STOP"
    parts.append(f"API:{api_state}")
    parts.append(f"CAM:{'RUN' if _is_proc_running(cam3d_proc) else 'STOP'}")

    # Show selected COM (what will be passed to child processes)
    try:
        sel = port_var.get().strip()
    except Exception:
        sel = ""
    if sel:
        parts.append(f"COM(选择):{sel}")

    # Show API effective COM when available (queried from /version)
    try:
        if isinstance(_api_version_cache, dict):
            ep = str(_api_version_cache.get("effective_port") or "").strip()
            if ep:
                parts.append(f"COM(API):{ep}")
    except Exception:
        pass

    tool_status_var.set(" | ".join(parts))


def _poll_api_version_loop() -> None:
    """Background poll of /version (throttled), so UI can show API effective COM."""

    def _tick():
        def _worker():
            global _api_version_cache, _api_version_last_error

            if requests is None:
                return

            base = ""
            try:
                base = base_var.get().strip()
                if not base:
                    base = f"http://127.0.0.1:{(port_api_var.get().strip() or '8001')}"
                r = requests.get(base.rstrip("/") + "/version", timeout=0.8)
                if r.status_code == 200:
                    data = r.json() if hasattr(r, "json") else None
                    if isinstance(data, dict):
                        _api_version_cache = data
                        _api_version_last_error = ""
                else:
                    _api_version_last_error = f"http {r.status_code}"
            except Exception as e:
                _api_version_last_error = str(e)

            safe_after(_refresh_tool_status)

        threading.Thread(target=_worker, daemon=True).start()

        # reschedule
        try:
            root.after(2000, _tick)
        except Exception:
            pass

    try:
        root.after(800, _tick)
    except Exception:
        pass


def _poll_log_queue():
    try:
        while True:
            line = _log_q.get_nowait()
            log_text.configure(state="normal")
            log_text.insert("end", line + "\n")
            log_text.see("end")
            log_text.configure(state="disabled")
    except Exception:
        pass
    _refresh_tool_status()
    safe_after(lambda: root.after(150, _poll_log_queue))


def _start_api_server():
    global api_proc
    if actions.is_connected():
        messagebox.showwarning(
            "串口互斥",
            "GUI 当前已连接串口（开始调试已开启）。\n\n"
            "请先关闭‘开始调试’断开串口，再启动 API 服务。",
        )
        return
    if _is_proc_running(api_proc):
        return
    port_s = port_api_var.get().strip() or "8001"

    # Keep camera base default consistent with the API port we launch.
    try:
        base_var.set(f"http://127.0.0.1:{port_s}")
    except Exception:
        pass

    def _api_base() -> str:
        base = base_var.get().strip()
        if not base:
            base = f"http://127.0.0.1:{port_s}"
        return base

    def _api_is_alive(base: str) -> bool:
        if requests is None:
            return False
        try:
            r = requests.get(base.rstrip("/") + "/version", timeout=1.5)
            return r.status_code == 200
        except Exception:
            return False

    def _api_shutdown(base: str) -> bool:
        if requests is None:
            return False
        try:
            r = requests.post(base.rstrip("/") + "/shutdown", json={}, timeout=2.0)
            return r.status_code == 200
        except Exception:
            return False

    base = _api_base()
    if _api_is_alive(base):
        if messagebox.askyesno("API 已在运行", f"检测到 API 已在运行：\n{base}\n\n是否重启它？"):
            _api_shutdown(base)
            time.sleep(0.6)
        else:
            _enqueue_log("API", f"already running at {base}")
            return

    if getattr(sys, "frozen", False):
        argv = [sys.executable, "--run-api", "--host", "127.0.0.1", "--port", port_s]
    else:
        argv = [sys.executable, os.path.abspath(__file__), "--run-api", "--host", "127.0.0.1", "--port", port_s]
    serial_port = port_var.get().strip()
    extra_env = {}
    if serial_port:
        extra_env["OCRA_SERIAL_PORT"] = serial_port
    api_proc = _spawn_process("API", argv, extra_env=extra_env or None)


def _stop_api_server():
    global api_proc

    base = base_var.get().strip() or f"http://127.0.0.1:{(port_api_var.get().strip() or '8001')}"

    def _shutdown_http(*, no_zero: bool) -> bool:
        if requests is None:
            return False
        try:
            r = requests.post(base.rstrip("/") + "/shutdown", json={"no_zero": bool(no_zero)}, timeout=2.0)
            return r.status_code == 200
        except Exception:
            return False

    # Prefer graceful shutdown (will do回零 by default in API)
    if _shutdown_http(no_zero=False):
        _enqueue_log("API", "shutdown ok (with zero)")
        # If we started it, let the process exit naturally.
        api_proc = None
        return

    # Fallback: if we own the process, terminate it.
    if _is_proc_running(api_proc):
        _enqueue_log("API", "shutdown http failed; fallback terminate (may still zero)")
        _stop_process("API", api_proc)
        api_proc = None
        return

    _enqueue_log("API", "shutdown failed (no running pid tracked)")


def _close_api_server_no_zero():
    """Close API without physical zero (best-effort).

    It asks the API to disconnect without calling emergency_stop().
    """

    global api_proc
    base = base_var.get().strip() or f"http://127.0.0.1:{(port_api_var.get().strip() or '8001')}"

    def _shutdown_http(*, no_zero: bool) -> bool:
        if requests is None:
            return False
        try:
            r = requests.post(base.rstrip("/") + "/shutdown", json={"no_zero": bool(no_zero)}, timeout=2.0)
            return r.status_code == 200
        except Exception:
            return False

    if _shutdown_http(no_zero=True):
        _enqueue_log("API", "shutdown ok (no zero)")
        api_proc = None
        return

    # If http failed but we own the pid, ask before force-killing (may trigger zero).
    if _is_proc_running(api_proc):
        if messagebox.askyesno(
            "关闭失败",
            "无法通过 HTTP 优雅关闭 API（不回零）。\n\n"
            "是否强制结束该进程？\n\n"
            "注意：强制结束可能触发回零/急停。",
        ):
            _stop_process("API", api_proc)
            api_proc = None
        return

    _enqueue_log("API", "shutdown(no_zero) failed (no running pid tracked)")


def _start_camera_3d():
    global cam3d_proc
    base = base_var.get().strip()
    if not base:
        base = f"http://127.0.0.1:{(port_api_var.get().strip() or '8001')}"

    cfg_in = config_path_var.get().strip() or "py_action/config_orca.yaml"
    serial_port = port_var.get().strip()
    # Let the worker do the resolve + runtime config write exactly once.
    cfg = cfg_in

    cam_idx = camera_index_var.get().strip()
    if not cam_idx:
        cam_idx = "0"

    if getattr(sys, "frozen", False):
        self_argv = [sys.executable]
    else:
        self_argv = [sys.executable, os.path.abspath(__file__)]

    if _is_proc_running(cam3d_proc):
        return

    argv = [
        *self_argv,
        "--run-camera3d",
        "--base",
        base,
        "--config",
        cfg,
        "--camera",
        cam_idx,
        "--hand",
        "left",
        "--hz",
        "10",
        "--ema",
        "0.75",
    ]
    if serial_port:
        argv.extend(["--serial-port", serial_port])

    extra_env = {}
    if serial_port:
        extra_env["OCRA_SERIAL_PORT"] = serial_port
    cam3d_proc = _spawn_process("CAM3D", argv, extra_env=extra_env or None)


def _stop_camera_3d():
    global cam3d_proc
    _stop_process("CAM3D", cam3d_proc)
    cam3d_proc = None


row_btn = tk.Frame(tool_panel, bg=PANEL)
row_btn.pack(fill="x", padx=18, pady=(0, 12))

for _i in range(3):
    row_btn.grid_columnconfigure(_i, weight=1, uniform="tool_btn")

tk.Button(
    row_btn,
    text="启动 API",
    bg=BTN,
    fg="white",
    relief="flat",
    command=_start_api_server,
).grid(row=0, column=0, sticky="ew", padx=(0, 6), pady=(0, 8))

tk.Button(
    row_btn,
    text="停止 API 回零",
    bg=BTN_DANGER,
    fg="white",
    relief="flat",
    command=_stop_api_server,
).grid(row=0, column=1, sticky="ew", padx=6, pady=(0, 8))

tk.Button(
    row_btn,
    text="关闭 API 不回零",
    bg="#666666",
    fg="white",
    relief="flat",
    command=_close_api_server_no_zero,
).grid(row=0, column=2, sticky="ew", padx=(6, 0), pady=(0, 8))

tk.Button(
    row_btn,
    text="启动相机跟随",
    bg=BTN,
    fg="white",
    relief="flat",
    command=_start_camera_3d,
).grid(row=1, column=0, columnspan=2, sticky="ew", padx=(0, 6))

tk.Button(
    row_btn,
    text="停止相机",
    bg=BTN_DANGER,
    fg="white",
    relief="flat",
    command=_stop_camera_3d,
).grid(row=1, column=2, sticky="ew", padx=(6, 0))

tool_status_var = tk.StringVar(value="API:STOP | CAM:STOP")
tool_status = tk.Label(
    tool_panel,
    textvariable=tool_status_var,
    bg="#071007",
    fg=ACCENT_2,
    font=("Consolas", 11, "bold"),
    padx=12,
    pady=7,
)
tool_status.pack(fill="x", padx=18, pady=(0, 16))

log_panel = tk.Frame(home_frame, bg=PANEL)
log_panel.pack(fill="both", expand=True, pady=(18, 0))

log_head = tk.Frame(log_panel, bg=PANEL)
log_head.pack(fill="x", padx=18, pady=(14, 8))

tk.Label(
    log_head,
    text="运行日志",
    font=("微软雅黑", 13, "bold"),
    bg=PANEL,
    fg=FG,
).pack(side="left")

tk.Label(
    log_head,
    text="PROCESS OUTPUT",
    font=("Consolas", 9),
    bg=PANEL,
    fg=MUTED,
).pack(side="right")

log_frame = tk.Frame(log_panel, bg=PANEL)
log_frame.pack(fill="both", expand=True, padx=18, pady=(0, 18))

log_text = tk.Text(log_frame, height=10, wrap="word", bg="#111111", fg="#DDDDDD")
log_text.configure(state="disabled")
log_text.pack(side="left", fill="both", expand=True)

sb = tk.Scrollbar(log_frame, command=log_text.yview)
sb.pack(side="right", fill="y")
log_text.configure(yscrollcommand=sb.set)

root.after(150, _poll_log_queue)
_poll_api_version_loop()


# ======================
# 控制面板页：复用你现有布局
# ======================
top_bar = tk.Frame(panel_frame, bg=BG)
top_bar.pack(fill="x", padx=12, pady=5)


def back_to_config():
    debug_var.set(False)
    set_debug_mode(False)


tk.Button(
    top_bar,
    text="← 返回串口配置",
    font=("微软雅黑", 10),
    bg="#EEEEEE",
    relief="flat",
    command=back_to_config,
).pack(side="left")

tk.Label(
    top_bar,
    text="控制面板",
    font=("微软雅黑", 12, "bold"),
    bg=BG
).pack(side="right")


# 内容区：左控制 + 右位置监视
content_row = tk.Frame(panel_frame, bg=BG)
content_row.pack(fill="both", expand=True, padx=12, pady=(0, 2))

left_col = tk.Frame(content_row, bg=BG)
left_col.pack(side="left", fill="both", expand=True)

right_col = tk.Frame(content_row, bg=BG)
right_col.pack(side="right", fill="y", padx=(10, 0))
# 右侧动作区稍微加宽一些（不使用滚动）
right_col.config(width=280)
right_col.pack_propagate(False)


manual_frame = tk.Frame(left_col, bg=PANEL)
manual_frame.pack(pady=3, fill="both", expand=True)

tk.Label(
    manual_frame,
    text="手动控制（ID 1-17）",
    font=("微软雅黑", 12, "bold"),
    bg=PANEL
).pack(pady=3)

speed_row = tk.Frame(manual_frame, bg=PANEL)
speed_row.pack(fill="x", padx=8, pady=(0, 4))
tk.Label(speed_row, text="Speed：", bg=PANEL).pack(side="left")
speed_var = tk.IntVar(value=500)
tk.Spinbox(speed_row, from_=0, to=500, width=6, textvariable=speed_var).pack(side="left", padx=6)

# 过流阈值（mA）：0 表示关闭
tk.Label(speed_row, text="过流保护：", bg=PANEL).pack(side="left", padx=(10, 0))
oc_enable_var = tk.BooleanVar(value=bool(_oc_cfg_enabled))
tk.Checkbutton(speed_row, text="启用", variable=oc_enable_var, bg=PANEL).pack(side="left")

tk.Label(speed_row, text="默认阈值(mA)：", bg=PANEL).pack(side="left", padx=(6, 0))
oc_default_ma_var = tk.IntVar(value=int(round(float(_oc_cfg_default_ma))) if _oc_cfg_default_ma is not None else 0)
tk.Spinbox(speed_row, from_=0, to=3250, increment=50, width=6, textvariable=oc_default_ma_var).pack(side="left", padx=6)


def _apply_overcurrent_params(*_):
    try:
        enabled = bool(oc_enable_var.get())
    except Exception:
        enabled = False

    try:
        v = int(oc_default_ma_var.get())
    except Exception:
        v = 0
    v = max(0, min(3250, int(v)))
    limit_ma = float(v) if v > 0 else None

    # Prefer YAML latch/reset_ratio (consistent across GUI/API/camera). Fall back to safe defaults.
    latch = bool(_oc_cfg_latch)
    reset_ratio = float(_oc_cfg_reset_ratio)
    if not (0.0 < reset_ratio < 1.0):
        reset_ratio = 0.80

    # If enabled but no limits configured, treat as disabled (avoid confusing "enabled" state).
    has_any_limit = bool(_oc_cfg_by_id) or (limit_ma is not None)
    enabled = bool(enabled) and bool(has_any_limit)

    safe_call(
        lambda: actions.configure_overcurrent_protection(
            enabled=enabled,
            limit_ma=limit_ma,
            limit_ma_by_id=dict(_oc_cfg_by_id) if _oc_cfg_by_id else None,
            latch=bool(latch),
            reset_ratio=float(reset_ratio),
        )
    )

    safe_after(_refresh_oc_limit_labels)


def _reset_overcurrent():
    def _worker():
        try:
            set_status_temp_safe("🟡 正在复位过流…", ttl_ms=1500)
        except Exception:
            pass
        _suspend_polling("oc_reset")
        try:
            safe_call(lambda: actions.reset_overcurrent_trips(None))
            # Retry once with polling suspended to increase torque re-enable success rate.
            try:
                time.sleep(0.25)
            except Exception:
                pass
            safe_call(lambda: actions.reset_overcurrent_trips(None))
        finally:
            _resume_polling()
        try:
            set_status_temp_safe("🟢 已复位过流", ttl_ms=2000)
        except Exception:
            pass

    threading.Thread(target=_worker, daemon=True).start()


tk.Button(
    speed_row,
    text="复位过流",
    font=("微软雅黑", 9),
    bg="#EEEEEE",
    relief="flat",
    command=_reset_overcurrent,
).pack(side="left", padx=(8, 0))


def _apply_write_params(*_):
    safe_call(lambda: actions.set_write_params(speed=speed_var.get()))


speed_var.trace_add("write", _apply_write_params)
_apply_write_params()

oc_enable_var.trace_add("write", _apply_overcurrent_params)
oc_default_ma_var.trace_add("write", _apply_overcurrent_params)
_apply_overcurrent_params()


sliders_table = tk.Frame(manual_frame, bg=PANEL)
sliders_table.pack(fill="both", expand=True, padx=8, pady=(0, 8))
sliders_table.columnconfigure(0, weight=0)
sliders_table.columnconfigure(1, weight=0)
sliders_table.columnconfigure(2, weight=0)
sliders_table.columnconfigure(3, weight=0)
sliders_table.columnconfigure(4, weight=1)

servo_scales = {}
servo_dragging = {sid: False for sid in range(1, 18)}
servo_id_labels: dict[int, tk.Label] = {}
servo_current_labels: dict[int, tk.Label] = {}
servo_limit_labels: dict[int, tk.Label] = {}

# 舵机 ID -> 灵巧手关节名称
SERVO_NAME = {
    1: "食指远端",
    2: "食指近端",
    3: "食指侧摆",
    4: "中指侧摆",
    5: "小指侧摆",
    6: "小指远端",
    7: "小指近端",
    8: "中指远端",
    9: "中指近端",
    10: "无名指近端",
    11: "无名指远端",
    12: "无名指侧摆",
    13: "拇指侧摆",
    14: "拇指近端",
    15: "拇指根部",
    16: "拇指远端",
    17: "腕部关节",
}


# ======================
# 右侧：3D URDF 模型 / 运动检测
# ======================

# 3D viewer instance (lazy-started)
_viewer3d = None  # OrcaHandViewer3D | None

_last_pos_for_3d: dict[int, int] = {}
_last_motion_ts: dict[int, float] = {sid: 0.0 for sid in range(1, 18)}
MOTION_DELTA = 6
MOTION_HOLD_SEC = 0.25


def _note_motion_from_pos(sid: int, pos: int):
    prev = _last_pos_for_3d.get(sid)
    _last_pos_for_3d[sid] = pos
    if prev is None:
        return
    if abs(int(pos) - int(prev)) >= MOTION_DELTA:
        _last_motion_ts[sid] = time.time()


def _get_active_motion_sids() -> set[int]:
    """Return set of servo IDs that have moved recently (for highlighting)."""
    now = time.time()
    return {
        sid
        for sid in range(1, 18)
        if (now - float(_last_motion_ts.get(sid, 0.0))) <= MOTION_HOLD_SEC
    }


def _start_3d_viewer():
    """Launch the URDF 3D hand viewer in a background window."""
    global _viewer3d
    if urdf_viewer is None:
        messagebox.showwarning("缺少依赖", "3D 模型需要 yourdfpy / pyglet，请先安装：pip install -r .\\py_action\\requirements.txt")
        return
    if _viewer3d is not None and _viewer3d.is_running():
        return
    try:
        _viewer3d = urdf_viewer.OrcaHandViewer3D(
            urdf_type="right",
            width=540,
            height=460,
            title="OrcaHand 3D 模型",
        )
        _viewer3d.start()
        set_status_temp_safe("🟢 3D 模型已启动", ttl_ms=2000)
    except Exception as e:
        _viewer3d = None
        messagebox.showerror("3D 模型启动失败", f"无法启动 3D 模型：\n\n{e}")


def _stop_3d_viewer():
    global _viewer3d
    if _viewer3d is not None:
        _viewer3d.close()
        _viewer3d = None


def _feed_positions_to_3d_viewer(positions: dict[int, int]):
    global _viewer3d
    if _viewer3d is not None and _viewer3d.is_running():
        try:
            _viewer3d.update_servo_positions(positions)
        except Exception:
            pass


# ======================
# 右侧：手部状态可视化（不占用控制链路）
# ======================
handviz_frame = tk.Frame(right_col, bg=PANEL)
handviz_frame.pack(fill="x", expand=False, pady=(3, 6))

tk.Label(
    handviz_frame,
    text="手部状态",
    font=("微软雅黑", 11, "bold"),
    bg=PANEL,
).pack(pady=(6, 2))

handviz_canvas = tk.Canvas(
    handviz_frame,
    width=260,
    height=210,
    bg="#071007",
    highlightthickness=0,
)
handviz_canvas.pack(fill="x", padx=10, pady=(0, 4))

handviz_meta_var = tk.StringVar(value="简化状态视图 | 3D URDF 可独立启动")
tk.Label(
    handviz_frame,
    textvariable=handviz_meta_var,
    bg=PANEL,
    fg=MUTED,
    font=("微软雅黑", 8),
).pack(pady=(0, 6))

_handviz_positions: dict[int, int] = {sid: 2047 for sid in range(1, 18)}


def _handviz_norm(sid: int) -> float:
    try:
        pos = float(_handviz_positions.get(int(sid), 2047))
    except Exception:
        pos = 2047.0
    return max(0.0, min(1.0, abs(pos - 2047.0) / 2048.0))


def _handviz_angle_offset(sid: int, degrees: float = 16.0) -> float:
    try:
        pos = float(_handviz_positions.get(int(sid), 2047))
    except Exception:
        pos = 2047.0
    return max(-degrees, min(degrees, ((pos - 2047.0) / 2048.0) * degrees))


def _handviz_finger_points(base_x: float, base_y: float, base_angle: float, bends: list[float], lengths: tuple[int, int, int]):
    pts = [(base_x, base_y)]
    angle = math.radians(base_angle)
    for idx, length in enumerate(lengths):
        if idx > 0:
            angle -= math.radians(bends[min(idx - 1, len(bends) - 1)])
        x0, y0 = pts[-1]
        pts.append((x0 + math.cos(angle) * length, y0 - math.sin(angle) * length))
    return pts


def _redraw_hand_status():
    try:
        c = handviz_canvas
        c.delete("all")
        w = max(240, int(c.winfo_width() or 260))
        h = max(200, int(c.winfo_height() or 210))

        active = _get_active_motion_sids()
        c.create_rectangle(0, 0, w, h, fill="#071007", outline="")
        c.create_line(16, h - 18, w - 16, h - 18, fill="#1D3B20")
        c.create_text(14, 14, text="SERVO MAP", anchor="nw", fill=ACCENT, font=("Consolas", 9, "bold"))

        wrist_shift = _handviz_angle_offset(17, 10.0)
        palm = [
            (w * 0.36 + wrist_shift, h * 0.78),
            (w * 0.26 + wrist_shift, h * 0.50),
            (w * 0.46 + wrist_shift, h * 0.38),
            (w * 0.70 + wrist_shift, h * 0.43),
            (w * 0.76 + wrist_shift, h * 0.70),
            (w * 0.56 + wrist_shift, h * 0.84),
        ]
        c.create_polygon(palm, fill="#112016", outline="#2F5D32", width=2)
        c.create_polygon(palm, fill="", outline=ACCENT, width=1)

        finger_specs = [
            ("食", (w * 0.35, h * 0.47), 105 + _handviz_angle_offset(3), [50 * _handviz_norm(2), 44 * _handviz_norm(1)], (34, 31, 24), [2, 1, 3]),
            ("中", (w * 0.47, h * 0.40), 93 + _handviz_angle_offset(4), [50 * _handviz_norm(9), 44 * _handviz_norm(8)], (40, 35, 26), [9, 8, 4]),
            ("无", (w * 0.59, h * 0.42), 82 + _handviz_angle_offset(12), [50 * _handviz_norm(10), 44 * _handviz_norm(11)], (36, 32, 24), [10, 11, 12]),
            ("小", (w * 0.70, h * 0.49), 72 + _handviz_angle_offset(5), [48 * _handviz_norm(7), 40 * _handviz_norm(6)], (30, 27, 21), [7, 6, 5]),
            ("拇", (w * 0.31, h * 0.68), 158 + _handviz_angle_offset(13), [46 * _handviz_norm(14), 42 * _handviz_norm(16)], (30, 28, 23), [15, 14, 16, 13]),
        ]

        for label, base, base_angle, bends, lengths, ids in finger_specs:
            pts = _handviz_finger_points(base[0] + wrist_shift, base[1], base_angle, bends, lengths)
            flat = [coord for pt in pts for coord in pt]
            is_active = any(int(sid) in active for sid in ids)
            line_color = ACCENT_2 if is_active else "#D9E8D6"
            dot_color = ACCENT if is_active else "#7D8C7E"
            c.create_line(*flat, fill=line_color, width=4, capstyle="round", joinstyle="round")
            for x, y in pts:
                c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=dot_color, outline="#071007", width=1)
            c.create_text(pts[-1][0], pts[-1][1] - 10, text=label, fill=MUTED, font=("微软雅黑", 8, "bold"))

        if active:
            ids = " ".join(f"{sid:02d}" for sid in sorted(active))
            handviz_meta_var.set(f"运动中 ID: {ids}")
        else:
            handviz_meta_var.set("简化状态视图 | 3D URDF 可独立启动")
    except Exception:
        pass


def _feed_positions_to_status_view(positions: dict[int, int]):
    try:
        for sid, pos in (positions or {}).items():
            _handviz_positions[int(sid)] = int(pos)
        _redraw_hand_status()
    except Exception:
        pass


handviz_canvas.bind("<Configure>", lambda _e: _redraw_hand_status())
safe_after(_redraw_hand_status)


def _any_dragging() -> bool:
    return any(servo_dragging.values())


def _set_comm_mode_for_drag():
    actions.comm_mode = actions.COMM_MODE_MANUAL if _any_dragging() else actions.COMM_MODE_IDLE


def on_scale_press(sid: int, _event=None):
    servo_dragging[sid] = True
    _set_comm_mode_for_drag()


def on_scale_release(sid: int, _event=None):
    servo_dragging[sid] = False
    _set_comm_mode_for_drag()


def on_scale_change(sid: int, v):
    if not servo_dragging.get(sid, False):
        return
    if not actions.is_connected():
        return
    try:
        pos = int(float(v))
    except Exception:
        return
    # 统一走 actions.move_simultaneous，这样能触发过流保护逻辑（并避免手动写绕过保护）。
    safe_call(lambda: actions.move_simultaneous([(int(sid), int(pos))]))
    safe_call(lambda: _note_motion_from_pos(int(sid), int(pos)))
    safe_call(lambda: _feed_positions_to_status_view({int(sid): int(pos)}))


for sid in range(1, 18):
    # 让每一行按高度均分，填充手动控制区的竖向留白
    sliders_table.rowconfigure(sid, weight=1)

    _lbl = tk.Label(
        sliders_table,
        text=f"{sid:02d} {SERVO_NAME.get(sid, '')}".strip(),
        bg=PANEL,
        fg="#333333",
        width=12,
        anchor="w",
        font=("微软雅黑", 10),
    )
    _lbl.grid(row=sid, column=0, sticky="nsw", pady=2)
    servo_id_labels[int(sid)] = _lbl

    scale = tk.Scale(
        sliders_table,
        from_=0,
        to=4095,
        orient="horizontal",
        # 缩短滑块长度，给右侧电流显示留空间
        length=200,
        showvalue=True,
        width=8,
        sliderlength=16,
        font=("微软雅黑", 9),
        bg=PANEL,
        highlightthickness=0,
        command=lambda v, _sid=sid: on_scale_change(_sid, v),
    )
    scale.grid(row=sid, column=1, sticky="nw", pady=1)
    scale.bind("<ButtonPress-1>", lambda e, _sid=sid: on_scale_press(_sid, e))
    scale.bind("<ButtonRelease-1>", lambda e, _sid=sid: on_scale_release(_sid, e))
    servo_scales[sid] = scale

    _cur = tk.Label(
        sliders_table,
        text="-- mA",
        bg=PANEL,
        fg="#666666",
        width=8,
        anchor="e",
        font=("微软雅黑", 10),
    )
    _cur.grid(row=sid, column=2, sticky="nse", padx=(6, 0), pady=2)
    servo_current_labels[int(sid)] = _cur

    _lim = tk.Label(
        sliders_table,
        text="限: --",
        bg=PANEL,
        fg="#888888",
        width=8,
        anchor="e",
        font=("微软雅黑", 10),
    )
    _lim.grid(row=sid, column=3, sticky="nse", padx=(6, 0), pady=2)
    servo_limit_labels[int(sid)] = _lim

    # filler (keeps layout stable)
    tk.Label(sliders_table, text="", bg=PANEL).grid(row=sid, column=4, sticky="nsew")


def _refresh_overcurrent_label_colors():
    """把触发过流保护的舵机 ID 标红。"""
    try:
        if hasattr(actions, "get_overcurrent_status"):
            st = actions.get_overcurrent_status()
        else:
            st = {}
        raw_ids = st.get("tripped_ids", [])
        if raw_ids is None:
            raw_ids = []
        if isinstance(raw_ids, (list, tuple, set)):
            tripped = {int(x) for x in raw_ids}
        else:
            tripped = set()
    except Exception:
        tripped = set()

    for sid, lbl in servo_id_labels.items():
        try:
            lbl.config(fg=("#FF6B6B" if int(sid) in tripped else FG))
        except Exception:
            pass

    # 同步'暂停/继续'按钮（过流自动暂停时也能及时反映）
    try:
        pb = globals().get("pause_btn")
        if pb is not None and hasattr(actions, "is_paused"):
            paused = bool(actions.is_paused())
            pb.config(text=("继续动作" if paused else "暂停（调试模式）"))
    except Exception:
        pass

    try:
        root.after(250, _refresh_overcurrent_label_colors)
    except Exception:
        pass


_refresh_overcurrent_label_colors()

# Initial render of per-servo limits
safe_after(_refresh_oc_limit_labels)


def _apply_scale_value_safe(sid: int, value: int):
    if servo_dragging.get(sid, False):
        return
    scale = servo_scales.get(sid)
    if scale is None:
        return
    safe_call(lambda: scale.set(int(value)))


def poll_all_positions_to_scales():
    last_connected = None
    poll_ids = list(range(1, 18))
    poll_idx = 0
    last_current_ts = 0.0
    while not closing_event.is_set():
        if polling_suspend_event.is_set():
            # failsafe: 防止因异常导致一直挂起，从而'读不到位置'
            try:
                in_action = bool(getattr(actions, "is_action_running", lambda: False)())
            except Exception:
                in_action = False
            if not in_action:
                try:
                    now = float(time.time())
                    if float(polling_suspend_ts) > 0 and (now - float(polling_suspend_ts)) > 3.0:
                        _resume_polling()
                except Exception:
                    pass
            time.sleep(0.2)
            continue
        connected = actions.is_connected()
        if last_connected is None or connected != last_connected:
            def apply_state():
                state = "normal" if connected else "disabled"
                for s in servo_scales.values():
                    try:
                        s.config(state=state)
                    except Exception:
                        pass
                if not connected:
                    for lbl in servo_current_labels.values():
                        try:
                            lbl.config(text="-- mA")
                        except Exception:
                            pass

            safe_after(apply_state)
            last_connected = connected

        # 实时回读：动作执行中也读当前位置（STS 内存表 0x38，长度 2）
        # 使用 SYNCREAD（0x82）同步读，效率更高、更稳。
        if connected and not _any_dragging():
            in_action = actions.comm_mode == actions.COMM_MODE_ACTION
            # 动作期间分批同步读，减少单次占锁时间；空闲时整批同步读
            batch = 8 if in_action else 17

            batch_ids = []
            for _ in range(batch):
                sid = poll_ids[poll_idx]
                poll_idx = (poll_idx + 1) % len(poll_ids)
                batch_ids.append(sid)

            # 动作期间避免阻塞写入：抢不到锁就跳过本轮回读
            lock_timeout = 0.004 if in_action else 0.15
            got_lock = False
            try:
                got_lock = actions.bus_lock.acquire(timeout=lock_timeout)
                if not got_lock:
                    time.sleep(0.02)
                    continue

                bus_obj = getattr(actions, "bus", None)
                if bus_obj is None or (not hasattr(bus_obj, "sync_read")):
                    results = {}
                    results_cur = {}
                    time.sleep(0.02)
                    continue

                # 同步读回：返回 dict[id]=payload(bytes) 或 None
                payloads = bus_obj.sync_read(
                    batch_ids,
                    start_addr=0x38,
                    data_len=2,
                    max_wait=(0.03 if in_action else 0.06),
                )

                results = {}
                for sid, payload in payloads.items():
                    if not payload or len(payload) != 2:
                        results[sid] = None
                    else:
                        results[sid] = payload[0] | (payload[1] << 8)

                # 电流回读：0x45 (len=2) -> mA
                results_cur: dict[int, float | None] = {}
                now_ts = time.time()
                cur_period = 0.25 if in_action else 0.18
                if (now_ts - float(last_current_ts)) >= float(cur_period):
                    last_current_ts = float(now_ts)
                    try:
                        if hasattr(bus_obj, "sync_read_current_ma"):
                            cur_map = bus_obj.sync_read_current_ma(
                                batch_ids,
                                max_wait=(0.04 if in_action else 0.08),
                            )
                            if isinstance(cur_map, dict):
                                for sid in batch_ids:
                                    results_cur[int(sid)] = cur_map.get(int(sid))
                    except Exception:
                        results_cur = {}
            except Exception:
                results = {}
                results_cur = {}
            finally:
                if got_lock:
                    try:
                        actions.bus_lock.release()
                    except Exception:
                        pass

            def apply_ui():
                for sid, pos in results.items():
                    if pos is None:
                        continue
                    _apply_scale_value_safe(int(sid), int(pos))
                    safe_call(lambda _sid=int(sid), _pos=int(pos): _note_motion_from_pos(_sid, _pos))

                # 更新电流显示（只更新本批次，减少 UI 刷新量）
                try:
                    for sid, ma in (results_cur or {}).items():
                        lbl = servo_current_labels.get(int(sid))
                        if lbl is None:
                            continue
                        if ma is None:
                            lbl.config(text="-- mA")
                        else:
                            lbl.config(text=f"{float(ma):.0f} mA")
                except Exception:
                    pass

                # Feed latest positions to 3D URDF viewer
                valid_pos = {int(s): int(p) for s, p in results.items() if p is not None}
                if valid_pos:
                    safe_call(lambda vp=valid_pos: _feed_positions_to_3d_viewer(vp))
                    safe_call(lambda vp=valid_pos: _feed_positions_to_status_view(vp))

            safe_after(apply_ui)

            # 空闲：更快刷新；动作：更高频但不抢占总线
            time.sleep(0.04 if in_action else 0.14)
        else:
            time.sleep(0.2)


threading.Thread(target=poll_all_positions_to_scales, daemon=True).start()


# ======================
# 右侧：预设动作（每行一个）
# ======================
action_scroll = tk.Frame(right_col, bg=PANEL)
action_scroll.pack(fill="both", expand=True, pady=3)

action_canvas = tk.Canvas(action_scroll, bg=PANEL, highlightthickness=0)
action_vbar = tk.Scrollbar(action_scroll, orient="vertical", command=action_canvas.yview)
action_canvas.configure(yscrollcommand=action_vbar.set)

action_vbar.pack(side="right", fill="y")
action_canvas.pack(side="left", fill="both", expand=True)

action_frame = tk.Frame(action_canvas, bg=PANEL)
_action_window = action_canvas.create_window((0, 0), window=action_frame, anchor="nw")


def _action_scroll_update_region(_e=None):
    try:
        action_canvas.configure(scrollregion=action_canvas.bbox("all"))
        action_canvas.itemconfigure(_action_window, width=action_canvas.winfo_width())
    except Exception:
        pass


def _action_scroll_sync_width(_e=None):
    try:
        action_canvas.itemconfigure(_action_window, width=action_canvas.winfo_width())
    except Exception:
        pass


def _action_scroll_on_mousewheel(event):
    try:
        delta = int(event.delta)
    except Exception:
        delta = 0
    if delta == 0:
        return
    action_canvas.yview_scroll(-1 if delta > 0 else 1, "units")
    return "break"


def _bind_action_scroll(_e=None):
    try:
        action_canvas.focus_set()
    except Exception:
        pass


def _unbind_action_scroll(_e=None):
    try:
        pass
    except Exception:
        pass


action_frame.bind("<Configure>", _action_scroll_update_region)
action_canvas.bind("<Configure>", _action_scroll_sync_width)
action_canvas.bind("<Enter>", _bind_action_scroll)
action_canvas.bind("<Leave>", _unbind_action_scroll)
action_frame.bind("<Enter>", _bind_action_scroll)
action_frame.bind("<Leave>", _unbind_action_scroll)
action_canvas.bind("<MouseWheel>", _action_scroll_on_mousewheel)
action_frame.bind("<MouseWheel>", _action_scroll_on_mousewheel)

tk.Label(
    action_frame,
    text="预设动作",
    font=("微软雅黑", 12, "bold"),
    bg=PANEL
).pack(pady=5)


def action_btn(text, func):
    tk.Button(
        action_frame,
        text=text,
        font=font_btn,
        bg=BTN,
        fg="white",
        relief="flat",
        height=2,
        command=lambda: actions.run_action(func, set_status_safe)
    ).pack(fill="x", padx=10, pady=4)


def _action_7_auto_calibrate_current():
    # 读 UI 输入（不要在工作线程里读 Tk 变量）
    try:
        margin_ma = float((calib_margin_var.get() or "").strip())
    except Exception:
        messagebox.showerror("参数错误", "动作7：加值(mA) 必须是数字。")
        return
    if margin_ma < 0 or margin_ma > 500:
        messagebox.showerror("参数错误", "动作7：加值(mA) 建议范围 0~500。")
        return

    if not actions.is_connected():
        messagebox.showwarning("未连接", "请先打开‘开始调试’并连接串口后再执行电流自动校准。")
        return

    # Use the same config path as toolbox (so API/camera/GUI stay consistent).
    cfg_in = config_path_var.get().strip() or "py_action/config_orca.yaml"
    resolved = _resolve_existing_config_path(cfg_in)
    if not os.path.exists(resolved):
        messagebox.showerror("配置文件不存在", f"找不到配置文件：\n\n{resolved}")
        return

    if not messagebox.askyesno(
        "自动校准电流",
        "将执行‘动作2 握拳’共 5 次，采样每个舵机握拳过程中的最大电流，并写入 YAML：\n"
        f"overcurrent_protection.per_servo_limit_ma = max(mA) + {margin_ma:.0f}\n\n"
        "注意：校准过程中不要手动拖动滑块/执行其它动作。\n\n继续吗？",
    ):
        return

    # Best-effort persist margin so next launch restores it.
    try:
        _write_overcurrent_calibration_margin_to_yaml(resolved, float(margin_ma))
    except Exception as e:
        try:
            messagebox.showwarning("写入加值失败", f"无法把动作7加值写入 YAML（将继续执行校准）：\n\n{e}")
        except Exception:
            pass

    def _worker():
        # Keep UI responsive.
        set_status_safe("🟣 电流校准：准备中")

        # Reduce bus contention while calibrating.
        _suspend_polling("action7")

        # Ensure no pause state blocks actions.
        try:
            actions.set_paused(False)
        except Exception:
            pass

        try:
            # 先采样"原始最大电流"(不加值)，再由 GUI 按当前加值计算最终阈值。
            raw_max_by_id = actions.calibrate_overcurrent_max_currents_fist_repeats(
                repeats=5,
                servo_ids=None,
                sample_interval_sec=0.05,
                lock_timeout_sec=0.01,
                status_cb=set_status_safe,
            )

            if not raw_max_by_id:
                set_status_safe("🟠 电流校准：未读到电流")
                safe_after(lambda: messagebox.showwarning("电流校准", "未读到有效电流数据（请确认固件支持电流寄存器 0x45，且通讯正常）。"))
                return

            # 计算最终阈值 = raw_max + margin
            limit_by_id: dict[int, float] = {}
            for sid, mx in raw_max_by_id.items():
                try:
                    limit_by_id[int(sid)] = float(mx) + float(margin_ma)
                except Exception:
                    continue

            # 落盘：原始 max + 当前加值（便于之后只改加值也能重新计算）
            _write_overcurrent_calibration_max_to_yaml(resolved, raw_max_by_id)

            out_path = _write_overcurrent_limits_to_yaml(resolved, limit_by_id)

            # Reload YAML and apply to runtime overcurrent config/UI.
            safe_after(_reload_oc_config_from_yaml)
            safe_after(_apply_overcurrent_params)
            set_status_safe("🟢 电流校准：已写入 YAML")

            def _msg():
                # Show a compact summary (IDs sorted).
                items = sorted((int(k), float(v)) for k, v in limit_by_id.items())
                preview = "\n".join([f"ID{sid}: {v:.0f} mA" for sid, v in items[:10]])
                more = "" if len(items) <= 10 else f"\n... 还有 {len(items) - 10} 个"  # avoid huge dialogs
                messagebox.showinfo(
                    "电流校准完成",
                    f"已写入：\n{out_path}\n\nper_servo_limit_ma 预览：\n{preview}{more}",
                )

            safe_after(_msg)
        except Exception as e:
            msg = str(e)
            try:
                tb = traceback.format_exc()
            except Exception:
                tb = ""
            if "aborted" in msg.lower():
                set_status_safe("🟡 电流校准：已中断")
                safe_after(lambda: messagebox.showinfo("电流校准", "已急停中断电流校准。"))
            else:
                set_status_safe("🔴 电流校准失败")
                detail = msg
                if tb:
                    detail = f"{msg}\n\n{tb}"
                safe_after(lambda detail=detail: messagebox.showerror("电流校准失败", detail))
        finally:
            _resume_polling()

    threading.Thread(target=_worker, daemon=True).start()


action_btn("动作 1：手指遍历", actions.action_1_finger_traverse)
action_btn("动作 2：握拳", actions.action_2_fist)
action_btn("动作 3：前后摆", actions.action_3_swing)
action_btn("动作 4：对指遍历", actions.action_4_pinch_traverse)
action_btn("动作 5：左右摆", actions.action_5_left_right)
action_btn("动作 6：仅回零", actions.action_6_zero_only)

# 动作7参数
action7_param_row = tk.Frame(action_frame, bg=PANEL)
action7_param_row.pack(fill="x", padx=10, pady=(8, 2))
tk.Label(
    action7_param_row,
    text="动作7 加值(mA)：",
    bg=PANEL,
    fg="#333333",
    font=("微软雅黑", 10),
).pack(side="left")
action7_margin_spin = tk.Spinbox(
    action7_param_row,
    from_=0,
    to=500,
    increment=1,
    textvariable=calib_margin_var,
    width=8,
    font=("微软雅黑", 10),
)
action7_margin_spin.pack(side="left")


# 动作7加值：自动保存（失焦/回车/修改后短暂防抖）
_action7_margin_save_job = None


def _persist_action7_margin_best_effort() -> None:
    try:
        cfg_in = (config_path_var.get() or "").strip() or "py_action/config_orca.yaml"
        resolved = _resolve_existing_config_path(cfg_in)
        if not os.path.exists(resolved):
            try:
                set_status_safe(f"🟠 动作7加值未保存：找不到配置 {resolved}")
            except Exception:
                pass
            return
        try:
            margin_ma = float((calib_margin_var.get() or "").strip())
        except Exception:
            try:
                set_status_safe("🟠 动作7加值未保存：不是数字")
            except Exception:
                pass
            return
        if margin_ma < 0:
            margin_ma = 0.0
        if margin_ma > 500:
            margin_ma = 500.0
        _write_overcurrent_calibration_margin_to_yaml(resolved, float(margin_ma))

        # 如果有"原始最大值"，则同步更新 per_servo_limit_ma = raw_max + margin
        try:
            raw_max = _read_overcurrent_calibration_max_ma_by_id(resolved)
            if raw_max:
                eff: dict[int, float] = {}
                for sid, mx in raw_max.items():
                    try:
                        eff[int(sid)] = float(mx) + float(margin_ma)
                    except Exception:
                        continue
                if eff:
                    _write_overcurrent_limits_to_yaml(resolved, eff)
        except Exception:
            pass

        # Refresh UI limit labels
        try:
            safe_after(_reload_oc_config_from_yaml)
        except Exception:
            pass
        # 临时状态提示（几秒后自动回到空闲）
        try:
            set_status_temp_safe(f"已保存动作7加值：{margin_ma:.0f} mA → {resolved}", ttl_ms=2500)
        except Exception:
            pass
    except Exception as e:
        try:
            set_status_temp_safe(f"动作7加值保存失败：{e}", ttl_ms=4000)
        except Exception:
            pass
        return


def _schedule_persist_action7_margin(*_):
    global _action7_margin_save_job
    try:
        # 如果当前是从 YAML/程序同步变量，避免触发自动保存回环。
        try:
            if bool(globals().get("_suppress_action7_margin_autosave")):
                return
        except Exception:
            pass
        if _action7_margin_save_job is not None:
            try:
                root.after_cancel(_action7_margin_save_job)
            except Exception:
                pass
        _action7_margin_save_job = root.after(500, _persist_action7_margin_best_effort)
    except Exception:
        pass


try:
    # Save on change (debounced)
    calib_margin_var.trace_add("write", _schedule_persist_action7_margin)
except Exception:
    pass

try:
    # Save immediately on Enter / losing focus
    action7_margin_spin.bind("<Return>", lambda _e: _persist_action7_margin_best_effort())
    action7_margin_spin.bind("<FocusOut>", lambda _e: _persist_action7_margin_best_effort())
except Exception:
    pass

tk.Button(
    action_frame,
    text="动作 7：自动校准电流(可设置加值)",
    font=font_btn,
    bg="#6A5ACD",
    fg="white",
    relief="flat",
    height=2,
    command=_action_7_auto_calibrate_current,
).pack(fill="x", padx=10, pady=6)


# 暂停/继续（用于动作过程中的调试与标定，不影响手动滑块写入）
def toggle_pause_action():
    try:
        paused = not bool(getattr(actions, "is_paused", lambda: False)())
        safe_call(lambda: actions.set_paused(paused))

        if paused:
            safe_call(lambda: pause_btn.config(text="继续动作"))
            set_status_safe("⏸ 已暂停（动作）")
        else:
            safe_call(lambda: pause_btn.config(text="暂停（调试模式）"))
            if safe_call(lambda: actions.is_action_running()) and actions.is_connected():
                set_status_safe("🔵 正在执行")
            else:
                set_status_safe("🟢 空闲")
    except Exception:
        pass


def disconnect_from_panel():
    """控制面板里的主动断开：
    - 非暂停：沿用'急停回零并断开'的习惯
    - 暂停（调试模式）：只断开，不回零，便于标定
    """
    try:
        paused = bool(getattr(actions, "is_paused", lambda: False)())
        if not paused:
            safe_call(lambda: actions.emergency_stop(set_status_safe))
        safe_call(lambda: actions.disconnect())
        safe_call(lambda: actions.set_paused(False))
        safe_call(lambda: pause_btn.config(text="暂停（调试模式）"))
        debug_var.set(False)
        set_status_safe("🔴 已断开")
        # 保持在控制面板页，便于继续查看/记录当前参数
    except Exception:
        pass


# 暂停（调试模式）+ 主动断开：同一行（将会放在急停上面）
bottom_row = tk.Frame(panel_frame, bg=BG)

pause_btn = tk.Button(
    bottom_row,
    text="暂停（调试模式）",
    font=("微软雅黑", 10, "bold"),
    bg="#FFA000",
    fg="white",
    relief="flat",
    height=1,
    command=toggle_pause_action,
)
pause_btn.pack(side="left", fill="x", expand=True, padx=(0, 6))

disconnect_btn = tk.Button(
    bottom_row,
    text="断开串口",
    font=("微软雅黑", 10, "bold"),
    bg="#EEEEEE",
    fg="#333333",
    relief="flat",
    height=1,
    command=disconnect_from_panel,
)
disconnect_btn.pack(side="left", fill="x", expand=True)


# 急停（固定贴底）
emergency_btn = tk.Button(
    panel_frame,
    text="立刻回零（急停）",
    font=("微软雅黑", 12, "bold"),
    bg=BTN_DANGER,
    fg="white",
    relief="flat",
    height=2,
)
emergency_btn.config(
    command=lambda: (
        safe_call(lambda: actions.set_paused(False)),
        safe_call(lambda: pause_btn.config(text="暂停（调试模式）")),
        actions.emergency_stop(set_status_safe),
    )
)

# pack 顺序很关键：先 pack 急停，再 pack 底部行，这样急停永远最底下
emergency_btn.pack(
    side="bottom",
    fill="x",
    padx=12,
    pady=6,
)

bottom_row.pack(
    side="bottom",
    fill="x",
    padx=12,
    pady=(0, 4),
)


_style_widget_tree(root)
safe_after(_redraw_hand_status)
set_status("🟢 空闲")
show_frame(config_frame)
root.mainloop()
