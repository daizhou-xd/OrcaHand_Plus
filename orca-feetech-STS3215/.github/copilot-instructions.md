# Copilot 代码库指引（ocra/ws）

## 仓库结构（多个相互独立的子项目）
- `orca_core-main/`：ORCA Hand 的核心控制库（Dynamixel）+ FastAPI API。
  - 核心类：`OrcaHand`/`MockOrcaHand` 在 `orca_core-main/orca_core/core.py`。
  - 真实硬件通信：`orca_core-main/orca_core/hardware/dynamixel_client.py`（依赖 `dynamixel-sdk`）。
  - Mock 通信：`orca_core-main/orca_core/hardware/mock_dynamixel_client.py`（单测用）。
  - 模型/配置：`orca_core-main/orca_core/models/<model>/config.yaml` + `calibration.yaml`。
  - API 服务：`orca_core-main/orca_core/api/api.py`（`/config`、`/connect`、`/status`、`/joints/position` 等）。
- `py_action/`：Windows 侧“灵巧手调试”GUI（Tkinter）+ 一个“兼容 orca_core 路径”的 FastAPI 服务（SMS/STS 舵机）。
  - GUI 入口：`py_action/gui/app.py`（会独占串口）。
  - API 入口：`py_action/api/server.py`（带 `POST /shutdown` 和 best-effort 串口释放）。
  - 串口总线协议：`py_action/servo_bus.py`；动作/线程/急停：`py_action/actions.py`。
- `orca_retargeter-main/`：手势/点云到 ORCA 手部关节/腱空间的 retarget（Torch + pytorch-kinematics）。
  - 入口类：`orca_retargeter-main/orca_retargeter/retargeter.py`（读取 `config.yaml`/`hand_scheme.yaml`/`retargeter.yaml`）。
- `orcahand_description-main/`：URDF/MJCF + 资产（仿真/可视化/mesh 工具）。

## 常用开发命令（Windows/PowerShell，按子项目工作目录）
- 安装 `orca_core`（可编辑）：`Set-Location .\orca_core-main; pip install -e .`
- 跑 `orca_core` 脚本（脚本第一个参数是 model 目录）：
  - `python .\scripts\tension.py orca_core\models\orcahand_v1_right`
  - `python .\scripts\calibrate.py orca_core\models\orcahand_v1_right`
- 跑 `orca_core` 测试（tests 用 `unittest` 写的，但 `pytest` 也能收集）：
  - `Set-Location .\orca_core-main; python -m pytest -q`
  - 或：`Set-Location .\orca_core-main; python -m unittest discover -s tests`
- 跑 `py_action` GUI：在仓库根目录 `ws` 下：`python .\py_action\gui\app.py`
- 跑 `py_action` API（推荐用同一个解释器路径，避免多 venv 混用）：
  - `.\.venv\Scripts\python.exe .\py_action\api\server.py`
- 跑 `orca_core` API：`Set-Location .\orca_core-main; python -m uvicorn orca_core.api.api:app --host 127.0.0.1 --port 8000`
- 跑 `orca_retargeter`（依赖较重）：`pip install -e .\orca_retargeter-main`；示例：`python .\orca_retargeter-main\ret_demo.py`

## 项目内约定/易踩坑（请按现有模式改）
- **模型路径解析**：`orca_core` 的 `OrcaHand(model_path)` 会通过 `orca_core-main/orca_core/utils/utils.py:get_model_path()` 把相对路径解析到 package root；脚本通常传 `orca_core/models/...`（相对 `orca_core-main` 目录）。
- **YAML 语义**：`orca_core` 的“模型目录”必须同时包含 `config.yaml` 与 `calibration.yaml`；标定/张紧等流程会写回 `calibration.yaml`。
- **电机反向约定**：`orca_core` 的 `config.yaml: joint_to_motor_map` 里允许用负 ID 表示反向，初始化时会归一化成正 ID 并在内部记录 inversion。
- **API 对齐**：`py_action/api/server.py` 旨在复刻 `orca_core` 的接口路径与错误语义（例如“未连接/未配置”倾向用 409）；新增端点或改字段时，优先保持两边一致。
- **串口互斥**：不要同时运行 `py_action/gui/app.py` 与 `py_action/api/server.py`（都会占用串口）。`py_action` 提供 `POST /shutdown` 用于“误关终端后端口仍被占用”的兜底退出。
- **单位与映射**：`py_action` API 的关节输入明确是弧度（rad），关节名 key 必须与 `py_action/config_orca.yaml` 的 `joint_to_servo_map` 完全一致（通常是中文名）。`orca_core` 的 joint-space 数值应与其 `config.yaml` 的 ROM/neutral 同单位（代码不强制标注单位）。

## 修改代码时的优先入口
- 硬件控制/关节到电机映射：先看 `orca_core-main/orca_core/core.py`（`_joint_to_motor_pos` / `_motor_to_joint_pos`）。
- 串口协议/批量写：先看 `py_action/servo_bus.py`（SYNCREAD/WRITE）与 `py_action/actions.py`（bus_lock、急停、线程）。
- FastAPI 行为（状态码/字段/生命周期）：`orca_core-main/orca_core/api/api.py` 与 `py_action/api/server.py`。
