#!/usr/bin/env python
"""滑条控制面板 — 设计预定动作。

每个关节一个滑条，范围 = 标定限位（度）。
拖动或输入数字即发送位置指令。支持力矩开关、中立位。
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from orca_core import OrcaHand

SLIDER_SPEED = 250  # 滑条舵机转速


class HandSliderUI:
    def __init__(self, root: tk.Tk, hand: OrcaHand):
        self.hand = hand
        self.limits_raw = hand.config.joint_limits_dict
        self.joint_ids = hand.config.joint_ids
        self.motor_map = hand.config.joint_to_motor_map

        # 舵机提速
        if hasattr(self.hand._motor_client, '_default_speed'):
            self.hand._motor_client._default_speed = SLIDER_SPEED

        self.joint_vars: dict[str, tk.DoubleVar] = {}
        self.entry_widgets: dict[str, ttk.Entry] = {}
        self._updating = False
        self._read_thread: threading.Thread | None = None
        self._read_running = False

        self._build_ui(root)
        self._start_reader()

    # ------------------------------------------------------------------
    def _build_ui(self, root: tk.Tk):
        root.title("ORCA Hand — 关节滑条控制")
        root.geometry("620x780")
        root.resizable(True, True)

        # --- 顶部按钮栏 ---
        toolbar = ttk.Frame(root)
        toolbar.pack(fill=tk.X, padx=8, pady=(8, 2))

        ttk.Button(toolbar, text="开启力矩", command=self._enable_torque).pack(side=tk.LEFT, padx=3)
        ttk.Button(toolbar, text="关闭力矩", command=self._disable_torque).pack(side=tk.LEFT, padx=3)
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=2)
        ttk.Button(toolbar, text="中立位", command=self._go_neutral).pack(side=tk.LEFT, padx=3)
        ttk.Button(toolbar, text="读取当前位置", command=self._read_positions).pack(side=tk.LEFT, padx=3)

        # 连接状态
        self.status_label = ttk.Label(toolbar, text="● 已连接", foreground="green")
        self.status_label.pack(side=tk.RIGHT, padx=8)

        # --- 滑条区域（可滚动） ---
        canvas_frame = ttk.Frame(root)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        self.canvas = tk.Canvas(canvas_frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.sliders_frame = ttk.Frame(self.canvas)

        self.sliders_frame.bind("<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))

        self.canvas.create_window((0, 0), window=self.sliders_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.canvas.bind("<Enter>", lambda e: self._bind_mousewheel())
        self.canvas.bind("<Leave>", lambda e: self._unbind_mousewheel())

        # --- 每个关节一行 ---
        for joint in self.joint_ids:
            deg_min, deg_max = self.hand.config.get_joint_deg_limits(joint)
            deg_min, deg_max = round(deg_min, 1), round(deg_max, 1)
            motor = self.motor_map.get(joint, "?")

            row = ttk.Frame(self.sliders_frame)
            row.pack(fill=tk.X, pady=1)

            # 关节名
            name_lbl = ttk.Label(row, text=joint, width=13, anchor=tk.E)
            name_lbl.pack(side=tk.LEFT, padx=(0, 4))

            # 滑条 — 范围 = 标定限位
            var = tk.DoubleVar(value=180.0)
            self.joint_vars[joint] = var

            scale = ttk.Scale(
                row, from_=deg_min, to=deg_max, orient=tk.HORIZONTAL,
                variable=var,
                command=lambda v, j=joint: self._on_slider(j, v),
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

            # 数字输入框
            entry = ttk.Entry(row, width=7, justify=tk.RIGHT)
            entry.insert(0, "180.0")
            entry.pack(side=tk.LEFT, padx=(4, 2))
            entry.bind("<Return>", lambda e, j=joint, ent=entry: self._on_entry(j, ent))
            entry.bind("<FocusOut>", lambda e, j=joint, ent=entry: self._on_entry(j, ent))
            self.entry_widgets[joint] = entry

            # 电机 ID
            motor_lbl = ttk.Label(row, text=f"M{motor}", width=5, anchor=tk.W,
                                  foreground="gray")
            motor_lbl.pack(side=tk.LEFT)

            # 限位范围
            limit_lbl = ttk.Label(row, text=f"[{deg_min:.0f}°, {deg_max:.0f}°]",
                                width=16, anchor=tk.W, foreground="gray")
            limit_lbl.pack(side=tk.LEFT, padx=(2, 0))

            # 滑条变化 → 更新输入框
            var.trace_add("write",
                lambda *a, j=joint, ent=entry: self._update_entry(j, ent))

        # --- 底部状态栏 ---
        footer = ttk.Frame(root)
        footer.pack(fill=tk.X, padx=8, pady=(2, 6))
        self.hint_label = ttk.Label(footer, text="拖动滑条 / 输入数字 Enter | 滚轮翻页 | 关闭窗口退出",
                                    foreground="gray")
        self.hint_label.pack(side=tk.LEFT)

        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # 滑条回调
    # ------------------------------------------------------------------
    def _on_slider(self, joint: str, value):
        if self._updating:
            return
        try:
            angle = float(value)
            self.hand.set_joint_positions({joint: angle}, num_steps=1)
        except Exception as e:
            self.status_label.config(text=f"✕ 错误: {e}", foreground="red")

    def _update_entry(self, joint: str, entry: ttk.Entry):
        if self._updating:
            return
        val = self.joint_vars[joint].get()
        entry.delete(0, tk.END)
        entry.insert(0, f"{val:.1f}")

    def _on_entry(self, joint: str, entry: ttk.Entry):
        """输入框回车 → 设置关节角度。"""
        try:
            angle = float(entry.get().strip())
            deg_min, deg_max = self.hand.config.get_joint_deg_limits(joint)
            angle = max(deg_min, min(deg_max, angle))
            self._updating = True
            self.joint_vars[joint].set(angle)
            self._updating = False
            entry.delete(0, tk.END)
            entry.insert(0, f"{angle:.1f}")
            self.hand.set_joint_positions({joint: angle}, num_steps=1)
        except ValueError:
            pass  # 忽略非法输入

    # ------------------------------------------------------------------
    # 按钮动作
    # ------------------------------------------------------------------
    def _enable_torque(self):
        try:
            self.hand.enable_torque()
            if hasattr(self.hand._motor_client, '_default_speed'):
                self.hand._motor_client._default_speed = SLIDER_SPEED
            self.status_label.config(text="● 力矩已开启", foreground="green")
            self._read_positions()
        except Exception as e:
            self.status_label.config(text=f"✕ {e}", foreground="red")

    def _disable_torque(self):
        try:
            self.hand.disable_torque()
            self.status_label.config(text="○ 力矩已关闭", foreground="orange")
        except Exception as e:
            self.status_label.config(text=f"✕ {e}", foreground="red")

    def _go_neutral(self):
        try:
            self.hand.set_neutral_position(num_steps=40, step_size=0.015)
            self._read_positions()
            self.status_label.config(text="● 已到中立位", foreground="green")
        except Exception as e:
            self.status_label.config(text=f"✕ {e}", foreground="red")

    def _read_positions(self):
        """从硬件读取当前关节位置，更新所有滑条和输入框。"""
        try:
            pos = self.hand.get_joint_position().as_dict()
            self._updating = True
            for joint in self.joint_ids:
                if joint in pos and pos[joint] is not None:
                    val = round(pos[joint], 1)
                    self.joint_vars[joint].set(val)
                    if joint in self.entry_widgets:
                        self.entry_widgets[joint].delete(0, tk.END)
                        self.entry_widgets[joint].insert(0, f"{val:.1f}")
            self._updating = False
            self.status_label.config(text="● 已连接", foreground="green")
        except Exception as e:
            self.status_label.config(text=f"✕ 读取失败: {e}", foreground="red")

    # ------------------------------------------------------------------
    # 后台位置刷新
    # ------------------------------------------------------------------
    def _start_reader(self):
        self._read_running = True
        self._read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._read_thread.start()

    def _reader_loop(self):
        while self._read_running:
            time.sleep(2.0)
            if not self._read_running:
                break
            try:
                self._read_positions()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 鼠标滚轮
    # ------------------------------------------------------------------
    def _bind_mousewheel(self):
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _unbind_mousewheel(self):
        self.canvas.unbind_all("<MouseWheel>")

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    # ------------------------------------------------------------------
    def _on_close(self):
        self._read_running = False
        try:
            self.hand.set_neutral_position(num_steps=40, step_size=0.015)
        except Exception:
            pass
        self.hand.disconnect()
        root.destroy()


# ======================================================================
def main():
    import argparse
    p = argparse.ArgumentParser(description="ORCA Hand 滑条控制面板")
    p.add_argument("--config", type=str, default="config/config.yaml")
    p.add_argument("--mock", action="store_true")
    p.add_argument("--skip-init", action="store_true")
    args = p.parse_args()

    config_path = str(_PROJECT_ROOT / args.config) if not Path(args.config).is_absolute() else args.config

    if args.mock:
        from orca_core.hardware_hand import MockOrcaHand
        hand = MockOrcaHand(config_path=config_path)
    else:
        hand = OrcaHand(config_path=config_path)

    ok, msg = hand.connect()
    print(f"[connect] {msg}")
    if not ok:
        print("连接失败")
        return

    if not args.skip_init:
        hand.init_joints(move_to_neutral=True)

    global root
    root = tk.Tk()
    HandSliderUI(root, hand)
    root.mainloop()


if __name__ == "__main__":
    main()
