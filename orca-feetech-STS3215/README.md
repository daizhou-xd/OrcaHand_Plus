# Orca 灵巧手调试（Windows / SMS-STS 舵机）

这是一套**面向复现与调试**的 Windows 端工具：
- 一个 Tkinter 图形界面（GUI）：手动滑块控制、预设动作、过流保护与“一键急停回零”。
- 一个 FastAPI 服务（HTTP API）：给外部程序（相机/手势/脚本）调用，接口风格对齐 `orca_core`。
- 一个相机脚本：从摄像头/MediaPipe 手部关键点估计关节角度，并通过 HTTP API 驱动灵巧手。

> 串口互斥：**GUI 和 API 不能同时连接同一个串口**（两者都会占用串口）。

---

## 1. 你需要准备什么（零基础版）

### 1) 电脑与软件
- Windows 10/11
- Python 3.10+（推荐 3.11）
- 一根 USB 转串口（或控制板自带的串口），并安装对应驱动

### 2) 硬件前置设置（非常重要）
在运行本项目之前，建议先用舵机厂商/飞特（Feetech）的上位机工具完成：
- 舵机 **ID 设置**（1~17）
- 舵机 **零位设置为 2047**
- 波特率设置为 **1,000,000**（本项目默认）

本项目默认的关节到舵机 ID 对应关系在 [py_action/config_orca.yaml](py_action/config_orca.yaml) 的 `joint_to_servo_map` 里（关节名通常是中文）。

### 3) 飞特资料包（可选，但强烈建议）
为了避免在 GitHub 公共仓库中再分发第三方软件/手册，本仓库**不直接上传** `飞特舵机相关资料/`。

- 下载与放置方式见：[docs/feetech-resources.md](docs/feetech-resources.md)

---

## 2. 安装与运行（最短复现路径）

下面所有命令都在仓库根目录运行（也就是有 `py_action/` 的那个目录）。

### Step A：创建虚拟环境（推荐）
PowerShell：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
```

### Step B：安装依赖
```powershell
pip install -r .\py_action\requirements.txt
```

> 如果你只想用 GUI（不跑相机），也可以先装最小依赖：`pyserial` + `pyyaml`。但为了“傻瓜复现”，建议直接装完整依赖。

### Step C：启动 GUI
```powershell
python .\py_action\gui\app.py
```

进入 GUI 后：
1. 在“串口配置”页选择 COM 口（不知道是哪个就插拔设备看变化），点“测试”。
2. 勾选“开始调试（连接并进入控制面板）”。
3. 先把 Speed 调低，再尝试拖动滑块。
4. 遇到异常随时点“急停”（会回零并恢复手动控制）。

---

## 3. 运行 HTTP API（给外部程序调用）

> 提醒：API 会占用串口，所以和 GUI 二选一。

启动 API：
```powershell
python .\py_action\api\server.py
```

默认监听：`http://127.0.0.1:8001`

你可以用浏览器打开：
- `http://127.0.0.1:8001/docs`（Swagger 文档）

典型调用顺序（示例用 curl）：
```powershell
# 1) 载入配置（传配置文件路径字符串）
curl -X POST http://127.0.0.1:8001/config -H "Content-Type: application/json" -d '"py_action/config_orca.yaml"'

# 2) 连接串口
curl -X POST http://127.0.0.1:8001/connect -H "Content-Type: application/json" -d '{}'

# 3) 下发关节角度（单位：rad，key 必须与 joint_to_servo_map 的关节名一致）
curl -X POST http://127.0.0.1:8001/joints/position -H "Content-Type: application/json" -d '{"positions":{"食指近端":0.5,"拇指近端":0.2}}'
```

如果你误关了运行 API 的终端，但端口/串口仍被占用，可以用兜底接口停止它：
```powershell
curl -X POST http://127.0.0.1:8001/shutdown -H "Content-Type: application/json" -d '{}'
```

---

## 4. 相机/手势驱动（可选）

相机脚本会读取摄像头画面，用 MediaPipe 估计手部关键点，并把估计到的关节角通过 API 发给灵巧手。

使用前：请先启动 API（上一节）。

启动相机脚本：
```powershell
python .\py_action\vision\camera3d_hand_to_api.py --base http://127.0.0.1:8001 --config py_action/config_orca.yaml
```

常见注意：
- `opencv-python` 与 `mediapipe` 体积较大，首次安装时间可能较久。
- 摄像头权限/驱动异常会导致打不开摄像头。

---

## 5. 配置文件说明（你最可能要改的地方）

配置文件是 [py_action/config_orca.yaml](py_action/config_orca.yaml)。

你最常改的是：
- `port`：串口号（也可以保持 `AUTO`，或者通过环境变量指定，见下节）
- `joint_to_servo_map`：关节名 -> 舵机 ID
- `joint_scale`：关节角度（rad）到舵机位置（0..4095）的映射范围
- `overcurrent_protection`：过流保护阈值（mA）与校准参数

GUI 里有“动作 7：自动校准电流(可设置加值)”：
- 会执行 5 次“握拳”，采样每个舵机握拳过程的最大电流
- 写回 YAML：
  - `overcurrent_protection.calibration_max_ma_by_id`（原始峰值）
  - `overcurrent_protection.calibration_margin_ma`（加值）
  - `overcurrent_protection.per_servo_limit_ma = 峰值 + 加值`

---

## 6. 环境变量（可选）

有些场景你不想改 YAML，可以用环境变量覆盖：
- `OCRA_SERIAL_PORT=COM5`：指定串口
- `OCRA_BAUDRATE=1000000`：指定波特率
- `OCRA_TORQUE_ENABLE_ADDR=0x28`：扭矩使能寄存器地址（默认 0x28）

---

## 7. 常见问题（FAQ）

### Q1：提示“端口被占用/打不开串口”
- 确保 **GUI 与 API 不要同时运行**
- 关掉占用串口的其它软件（厂商上位机、其它脚本）
- 拔插 USB 设备，换 USB 口

### Q2：拖动滑块没反应
- 先点一次“急停”（会回零并恢复手动控制）
- 确认已经“开始调试”并连接成功
- 如果刚从控制面板返回串口配置页，请等待状态栏回到“空闲”再重新连接

### Q3：动作一运行就撞/很猛
- 先把 Speed 调小
- 再逐步修改 [py_action/config_orca.yaml](py_action/config_orca.yaml) 的映射范围

### Q4：过流保护频繁触发
- 先在 GUI 里运行“动作7 电流校准”，并适当增加“加值(mA)”
- 确认电源与机械结构没有卡滞

---

## 8. 代码结构（给想改代码的人）

- [py_action/gui/app.py](py_action/gui/app.py)：GUI 入口（滑块、动作按钮、状态栏、工具箱）
- [py_action/control/actions.py](py_action/control/actions.py)：动作与控制核心（串口互斥、急停回零、过流保护、校准流程）
- [py_action/hardware/servo_bus.py](py_action/hardware/servo_bus.py)：底层总线协议（SYNC READ/WRITE、位置/电流/扭矩寄存器）
- [py_action/api/server.py](py_action/api/server.py)：FastAPI 服务（对齐 orca_core 风格的接口）
- [py_action/vision/camera3d_hand_to_api.py](py_action/vision/camera3d_hand_to_api.py)：相机/手势到关节角，再到 HTTP API

---

## 免责声明与安全提示
- 本项目面向特定硬件/固件组合复现，其他舵机/协议可能无法直接使用。
- 第一次运行请让机械结构保持安全距离，速度调低，随时准备急停。

