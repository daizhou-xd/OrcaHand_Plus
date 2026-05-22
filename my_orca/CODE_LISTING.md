# My ORCA 项目代码清单

**项目名称：** My ORCA — 基于摄像头手势追踪的 ORCA Hand 遥操作抓取系统

**运行环境：** Windows 11, Python >= 3.10, Feetech ST3215 舵机, COM13 串口

---

## 一、新增开发模块（核心成果）

| 文件 | 行数 | 功能说明 |
|---|---|---|
| `hand_tracker.py` | 187 | 基于 MediaPipe Hands 的摄像头人手关键点追踪器。采集 21 个手部关键点（像素坐标 + 世界坐标），内建时序平滑滤波，支持实时可视化标注。 |
| `hand_mapper.py` | 322 | 人手关键点到 ORCA Hand 17 关节角度的映射器。实现拇指 4 关节（MCP/ABD/PIP/DIP）+ 四指 3 关节（ABD/MCP/PIP）+ 手腕的角度计算，含抓取状态检测。 |
| `main_grasp.py` | 301 | 主抓取演示程序。支持 4 种运行模式（摄像头遥操作/强力抓取/指尖捏取/循环切换），内建自动抓取检测、键盘快捷键、3 种预设抓取姿态。 |
| `calibrate.py` | 134 | 标定与腱绳张紧脚本。封装 ORCA Hand 的完整标定流程（连接舵机→驱动关节到机械限位→计算传动比→写入标定文件）。 |
| `config/config.yaml` | 199 | Feetech 右手硬件配置文件。定义 17 舵机 ID 映射、关节活动范围、中立位置、标定电流参数及标定序列（28 步）。 |
| `requirements.txt` | 10 | Python 依赖清单（mediapipe, opencv-python, pyserial 等）。 |

## 二、复用核心控制包（orca_core 移植）

| 文件 | 行数 | 功能说明 |
|---|---|---|
| `orca_core/__init__.py` | 24 | 包入口，导出 OrcaHand / OrcaJointPositions / OrcaHandConfig 等核心类。 |
| `orca_core/base_hand.py` | 177 | 抽象手部基类 BaseHand。定义插值运动（线性插值）、关节位置安全限幅（ROM clamping）、中立位置/零位控制、位置录制与回放。 |
| `orca_core/hardware_hand.py` | 1162 | OrcaHand 硬件手类。实现完整的硬件生命周期：连接管理（含端口自动检测）、力矩使能/禁用、多模式控制（位置/速度/电流/电流基位置）、自动标定流程（含腱绳张紧、Jitter）、电机↔关节双向转换、编码器 wrap 修正。 |
| `orca_core/joint_position.py` | 146 | 不可变关节位置数据容器 OrcaJointPositions。支持 dict/ndarray/list 互转，NaN 值过滤，默认关节排序注册。 |
| `orca_core/hand_config.py` | 325 | 配置加载与校验。从 YAML 读取硬件参数（电机映射含正反转标志、关节 ROM、标定序列），校验配置完整性（电机数量=关节数量、ROM 合法性等）。 |
| `orca_core/calibration.py` | 78 | 标定结果存储 CalibrationResult。记录电机限位（motor_limits）和关节-电机传动比（joint_to_motor_ratios）。 |
| `orca_core/constants.py` | 65 | 常量定义。控制模式映射（电流=0/速度=1/位置=3/多圈=4/电流基位置=5）、电机类型（dynamixel/feetech）、USB VID 识别表、标定关节类型枚举。 |
| `orca_core/core.py` | 11 | 核心模块重新导出。 |
| `orca_core/version.py` | 1 | 版本号 v2。 |

## 三、硬件驱动层（Feetech SCServo SDK）

| 文件 | 行数 | 功能说明 |
|---|---|---|
| `orca_core/hardware/__init__.py` | 0 | 包标记文件。 |
| `orca_core/hardware/motor_client.py` | 138 | 抽象电机客户端接口 MotorClient（ABC）。定义 connect/disconnect/set_torque_enabled/set_operating_mode/read_pos_vel_cur/read_temperature/write_desired_pos/write_desired_current 等 9 个抽象方法。 |
| `orca_core/hardware/feetech_client.py` | 562 | FeetechClient 电机客户端实现。使用 SCServo 协议通过串口控制 STS/SMS 舵机：同步读取位置/速度/电流/温度、同步写入目标位置（含加速度/速度/力矩参数）、力矩映射（电流 mA→torque 0-1000）、偏移标定（INST_OFSCAL 指令）、自动清理注册。 |
| `orca_core/hardware/feetech/__init__.py` | 10 | Feetech SDK 子包入口。 |
| `orca_core/hardware/feetech/scservo_def.py` | 27 | SCServo 协议常量。通信指令码（PING/READ/WRITE/SYNC_WRITE/SYNC_READ/OFSCAL）、通信结果码（SUCCESS/TX_FAIL/RX_TIMEOUT/RX_CORRUPT 等）。 |
| `orca_core/hardware/feetech/protocol_packet_handler.py` | 565 | SCServo 协议包处理器。实现数据包收发（含校验和验证、超时检测）、字节级读写（1/2/4 字节）、同步读写批次命令、偏移标定指令（reOfsCal）、复位指令。 |
| `orca_core/hardware/feetech/port_handler.py` | 114 | 串口处理器。基于 pyserial 实现串口打开/关闭/参数配置/读写/超时管理，支持 Windows/Linux 双平台。 |
| `orca_core/hardware/feetech/sms_sts.py` | 114 | SMS_STS 舵机协议封装。定义内存表地址（力矩使能/目标位置/速度/加速度/当前位置/速度/电流/温度），实现 WritePosEx/SyncWritePosEx 等高级指令。 |
| `orca_core/hardware/feetech/group_sync_write.py` | 73 | 同步写入组管理器。批量组织多个舵机的参数数据，通过 SYNC_WRITE 指令一次性发送。 |
| `orca_core/hardware/feetech/group_sync_read.py` | 151 | 同步读取组管理器。批量组织多个舵机的读取请求，从 SYNC_READ 返回包中解析各舵机数据。 |
| `orca_core/hardware/feetech/hls.py` | 114 | HLS 系列舵机协议封装（备用）。 |
| `orca_core/hardware/feetech/scscl.py` | 104 | SCSCL 系列舵机协议封装（备用）。 |
| `orca_core/hardware/dynamixel_client.py` | 790 | Dynamixel 电机客户端（备用，本项目使用 Feetech）。 |
| `orca_core/hardware/mock_dynamixel_client.py` | 521 | 模拟电机客户端。在内存中模拟舵机状态，用于无硬件测试（--mock 模式）。 |

## 四、工具与配置

| 文件 | 行数 | 功能说明 |
|---|---|---|
| `orca_core/utils/__init__.py` | 1 | 包标记文件。 |
| `orca_core/utils/utils.py` | 308 | 工具函数集。YAML 读写（update_yaml/read_yaml）、模型路径解析（自动匹配版本）、串口自动检测（USB VID 匹配）、交互式端口选择（curses 界面）、插值算法（线性/缓入缓出）。 |
| `orca_core/models/v1/orcahand_right/config.yaml` | 180 | v1 右手参考配置（本项目基于此版本）。 |
| `orca_core/models/v1/orcahand_left/config.yaml` | 148 | v1 左手参考配置。 |
| `orca_core/models/v2/orcahand_right/config.yaml` | 202 | v2 右手参考配置。 |
| `orca_core/models/v2/orcahand_left/config.yaml` | 202 | v2 左手参考配置。 |
| `orca_core/models/v2/orcahand_touch_right/config.yaml` | 208 | v2 触觉右手参考配置。 |
| `orca_core/models/v2/orcahand_touch_left/config.yaml` | 208 | v2 触觉左手参考配置。 |
| `orca_core/api/__init__.py` | 0 | API 包标记文件。 |
| `orca_core/api/api.py` | 365 | FastAPI Web API（开发中，本项目未使用）。 |

## 五、项目文档

| 文件 | 行数 | 功能说明 |
|---|---|---|
| `README.md` | 407 | 英文版项目文档。含目录结构、快速开始、配置说明、脚本参考、API 文档、故障排除。 |
| `README_cn.md` | 407 | 中文版项目文档。内容同上，中文撰写。 |

## 六、项目统计

| 类别 | 文件数 | 总行数 |
|---|---|---|
| 新增开发模块（核心成果） | 6 | ~1,153 |
| 核心控制包（orca_core 移植） | 8 | ~2,100 |
| Feetech 驱动层 | 10 | ~2,200 |
| Dynamixel 驱动层（备用） | 2 | ~1,311 |
| 工具与参考配置 | 8 | ~1,500 |
| 文档 | 2 | ~814 |
| **总计** | **36** | **~9,078** |

---

*生成日期：2026-05-05*
