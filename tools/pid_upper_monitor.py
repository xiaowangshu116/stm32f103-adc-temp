#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 PID 温控器 — 上位机串口调试界面
======================================
功能: 串口连接、温度曲线实时显示、PID 参数设定、
      模式切换、加热控制、数据日志

协议: $CMD=value\n 格式
依赖: Python 3.x + tkinter + pyserial
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import re
from collections import deque

# ====================== 全局配置 ======================
APP_TITLE  = "STM32 PID 温控器上位机 v2.0"
BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800]
DEFAULT_BAUD = 115200
CHART_SECONDS = 60       # 图表显示最近 60 秒
UPDATE_MS     = 200       # UI 刷新间隔 (ms)

# ====================== 串口读取线程 ======================
class SerialThread(threading.Thread):
    """后台线程: 持续读取串口数据, 放入队列供 UI 消费"""
    def __init__(self, ser, callback):
        super().__init__(daemon=True)
        self.ser = ser
        self.callback = callback
        self.running = True
        self.buffer = b""

    def run(self):
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    self.callback(data)
                else:
                    time.sleep(0.02)
            except (serial.SerialException, OSError):
                self.callback(None)  # 通知 UI 串口断开
                break

    def stop(self):
        self.running = False

# ====================== 温度曲线 Canvas ======================
class TempChart(tk.Canvas):
    """温度实时趋势图 (基于 tkinter Canvas 自绘)"""
    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg="#1a1a2e", highlightthickness=0, **kwargs)
        self.data = deque(maxlen=CHART_SECONDS)
        self.setpoint_data = deque(maxlen=CHART_SECONDS)
        self.bind("<Configure>", self._on_resize)

    def add_point(self, temp, setpoint):
        now = time.time()
        self.data.append((now, temp))
        self.setpoint_data.append((now, setpoint))
        self._redraw()

    def _on_resize(self, event=None):
        self._redraw()

    def _redraw(self):
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 50 or h < 50 or len(self.data) < 2:
            return

        margin = 40
        graph_w = w - margin - 10
        graph_h = h - margin - 15

        # 背景网格
        for i in range(5):
            y = margin + graph_h * i / 4
            self.create_line(margin, y, w - 10, y, fill="#2a2a4a", dash=(2, 4))
        for i in range(6):
            x = margin + graph_w * i / 5
            self.create_line(x, margin, x, h - 15, fill="#2a2a4a", dash=(2, 4))

        # 计算 Y 轴范围 (自动缩放)
        all_temps = [p[1] for p in self.data]
        all_sets  = [p[1] for p in self.setpoint_data]
        y_min = min(min(all_temps), min(all_sets)) - 2
        y_max = max(max(all_temps), max(all_sets)) + 2
        if y_max - y_min < 5:
            mid = (y_min + y_max) / 2
            y_min = mid - 5
            y_max = mid + 5

        def to_xy(t, v):
            x = margin + graph_w * (len(self.data) - 1 - (self.data[-1][0] - t)) / max(CHART_SECONDS, len(self.data))
            # Keep within bounds
            frac = (v - y_min) / max(y_max - y_min, 0.1)
            y = margin + graph_h * (1 - frac)
            return x, y

        # 绘制温度曲线
        points_temp = [to_xy(t, v) for t, v in self.data]
        if len(points_temp) >= 2:
            for i in range(len(points_temp) - 1):
                self.create_line(points_temp[i][0], points_temp[i][1],
                                 points_temp[i+1][0], points_temp[i+1][1],
                                 fill="#00ff88", width=2, smooth=True)

        # 绘制目标温度虚线
        points_set = [to_xy(t, v) for t, v in self.setpoint_data]
        if len(points_set) >= 2:
            for i in range(len(points_set) - 1):
                self.create_line(points_set[i][0], points_set[i][1],
                                 points_set[i+1][0], points_set[i+1][1],
                                 fill="#ff6b6b", width=1, dash=(4, 4), smooth=True)

        # Y 轴标签
        for i in range(5):
            v = y_min + (y_max - y_min) * i / 4
            self.create_text(8, to_xy(0, v)[1], text=f"{v:.0f}",
                             fill="#8888aa", font=("Consolas", 8), anchor="w")

        # 图例
        self.create_text(margin + 60, 10, text="── 实际温度", fill="#00ff88",
                         font=("微软雅黑", 9), anchor="w")
        self.create_text(margin + 160, 10, text="- - 目标温度", fill="#ff6b6b",
                         font=("微软雅黑", 9), anchor="w")

# ====================== 主应用 ======================
class App:
    def __init__(self, root):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("960x720")
        self.root.configure(bg="#0f0f1a")
        self.root.minsize(800, 600)

        # 状态变量
        self.ser = None
        self.serial_thread = None
        self.rx_buffer = ""
        self.current_temp = 0.0
        self.current_setpoint = 25.0
        self.current_power = 0
        self.heater_on = False
        self.connected = False
        self.mode = "AUTO"

        self._build_ui()
        self._refresh_ports()

    # =================== UI 构建 ===================
    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TButton", font=("微软雅黑", 9), padding=4)
        style.configure("TLabel", font=("微软雅黑", 10), background="#0f0f1a", foreground="#cccccc")
        style.configure("TEntry", font=("Consolas", 10))
        style.configure("TCombobox", font=("Consolas", 10))

        # 主布局: 左(图表) + 右(控制面板)
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg="#0f0f1a", sashwidth=3)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ----- 左侧: 温度图表 -----
        left_frame = tk.Frame(main_pane, bg="#0f0f1a")
        main_pane.add(left_frame, width=600)

        chart_label = tk.Label(left_frame, text="📈 温度趋势 (实时)", bg="#0f0f1a", fg="#cccccc",
                               font=("微软雅黑", 11, "bold"))
        chart_label.pack(anchor="w", pady=(0, 2))

        self.chart = TempChart(left_frame, width=580, height=380)
        self.chart.pack(fill=tk.BOTH, expand=True)

        # 大字体温度显示
        temp_frame = tk.Frame(left_frame, bg="#0f0f1a")
        temp_frame.pack(fill=tk.X, pady=5)
        self.temp_label = tk.Label(temp_frame, text="--.- °C", bg="#0f0f1a", fg="#00ff88",
                                   font=("Consolas", 36, "bold"))
        self.temp_label.pack(side=tk.LEFT, padx=10)
        info_frame = tk.Frame(temp_frame, bg="#0f0f1a")
        info_frame.pack(side=tk.LEFT, padx=20)
        self.setpoint_label = tk.Label(info_frame, text="目标: 25.0 °C", bg="#0f0f1a", fg="#ff6b6b",
                                       font=("微软雅黑", 12))
        self.setpoint_label.pack(anchor="w")
        self.power_label = tk.Label(info_frame, text="功率: 0%", bg="#0f0f1a", fg="#ffaa00",
                                    font=("微软雅黑", 12))
        self.power_label.pack(anchor="w")
        self.heater_label = tk.Label(info_frame, text="加热: OFF", bg="#0f0f1a", fg="#888888",
                                     font=("微软雅黑", 12))
        self.heater_label.pack(anchor="w")
        self.mode_label = tk.Label(info_frame, text="模式: AUTO", bg="#0f0f1a", fg="#88aaff",
                                   font=("微软雅黑", 12))
        self.mode_label.pack(anchor="w")

        # 数据日志
        log_label = tk.Label(left_frame, text="📋 数据日志", bg="#0f0f1a", fg="#cccccc",
                             font=("微软雅黑", 11, "bold"))
        log_label.pack(anchor="w", pady=(5, 2))
        self.log_area = scrolledtext.ScrolledText(left_frame, height=8, bg="#1a1a2e", fg="#aabbcc",
                                                  insertbackground="white", font=("Consolas", 9),
                                                  wrap=tk.WORD)
        self.log_area.pack(fill=tk.BOTH, expand=True)

        # ----- 右侧: 控制面板 -----
        right_frame = tk.Frame(main_pane, bg="#14142a", width=300)
        main_pane.add(right_frame, width=300)

        # 串口配置
        self._section_label(right_frame, "🔌 串口配置")
        port_frame = tk.Frame(right_frame, bg="#14142a")
        port_frame.pack(fill=tk.X, pady=2)
        self.port_combo = ttk.Combobox(port_frame, width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=2)
        self.baud_combo = ttk.Combobox(port_frame, values=BAUD_RATES, width=10, state="readonly")
        self.baud_combo.set(str(DEFAULT_BAUD))
        self.baud_combo.pack(side=tk.LEFT, padx=2)
        tk.Button(port_frame, text="⟳", bg="#2a2a4a", fg="#cccccc", relief=tk.FLAT,
                  command=self._refresh_ports, font=("Consolas", 10), width=2).pack(side=tk.LEFT)

        conn_frame = tk.Frame(right_frame, bg="#14142a")
        conn_frame.pack(fill=tk.X, pady=3)
        self.connect_btn = tk.Button(conn_frame, text="🔗 连接", bg="#006622", fg="white",
                                     font=("微软雅黑", 10, "bold"), relief=tk.FLAT, padx=10, pady=3,
                                     command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        self.disconnect_btn = tk.Button(conn_frame, text="⏹ 断开", bg="#662222", fg="white",
                                        font=("微软雅黑", 10), relief=tk.FLAT, padx=10, pady=3,
                                        state=tk.DISABLED, command=self._disconnect)
        self.disconnect_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)

        # 温度设定
        self._section_label(right_frame, "🌡️ 温度设定")
        set_frame = tk.Frame(right_frame, bg="#14142a")
        set_frame.pack(fill=tk.X, pady=2)
        tk.Label(set_frame, text="目标温度(°C):", bg="#14142a", fg="#cccccc",
                 font=("微软雅黑", 9)).pack(anchor="w")
        temp_row = tk.Frame(set_frame, bg="#14142a")
        temp_row.pack(fill=tk.X, pady=2)
        self.setpoint_entry = ttk.Entry(temp_row, width=14, font=("Consolas", 12))
        self.setpoint_entry.insert(0, "25.0")
        self.setpoint_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(temp_row, text="设定", bg="#005588", fg="white", font=("微软雅黑", 10),
                  relief=tk.FLAT, padx=12, pady=2, command=self._set_temp).pack(side=tk.LEFT, padx=2)

        # PID 参数
        self._section_label(right_frame, "⚙️ PID 参数")
        pid_frame = tk.Frame(right_frame, bg="#14142a")
        pid_frame.pack(fill=tk.X, pady=2)

        self.kp_entry = self._pid_row(pid_frame, "Kp (比例):", "8.0")
        self.ki_entry = self._pid_row(pid_frame, "Ki (积分):", "0.1")
        self.kd_entry = self._pid_row(pid_frame, "Kd (微分):", "2.0")

        tk.Button(pid_frame, text="更新 PID 参数", bg="#553300", fg="white",
                  font=("微软雅黑", 10), relief=tk.FLAT, padx=10, pady=3,
                  command=self._set_pid).pack(fill=tk.X, pady=4)

        # 模式切换
        self._section_label(right_frame, "🎛️ 模式")
        mode_frame = tk.Frame(right_frame, bg="#14142a")
        mode_frame.pack(fill=tk.X, pady=2)
        self.auto_btn = tk.Button(mode_frame, text="🔵 AUTO 自动", bg="#004488", fg="white",
                                  font=("微软雅黑", 10, "bold"), relief=tk.FLAT, padx=8, pady=3,
                                  command=self._set_auto)
        self.auto_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        self.manual_btn = tk.Button(mode_frame, text="🟡 MAN 手动", bg="#444444", fg="#aaaaaa",
                                    font=("微软雅黑", 10), relief=tk.FLAT, padx=8, pady=3,
                                    command=self._set_manual)
        self.manual_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)

        # 手动加热控制
        heat_frame = tk.Frame(right_frame, bg="#14142a")
        heat_frame.pack(fill=tk.X, pady=3)
        tk.Button(heat_frame, text="🔥 开加热", bg="#662200", fg="white", font=("微软雅黑", 10),
                  relief=tk.FLAT, padx=8, pady=3, command=lambda: self._send_cmd("$HEAT=1")).pack(
                  side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        tk.Button(heat_frame, text="❄ 关加热", bg="#224466", fg="white", font=("微软雅黑", 10),
                  relief=tk.FLAT, padx=8, pady=3, command=lambda: self._send_cmd("$HEAT=0")).pack(
                  side=tk.LEFT, padx=2, fill=tk.X, expand=True)

        # 快捷查询按钮
        self._section_label(right_frame, "📡 快捷查询")
        qf = tk.Frame(right_frame, bg="#14142a")
        qf.pack(fill=tk.X, pady=2)
        for label, cmd in [("状态", "$STATUS"), ("PID", "$PID?"), ("温度", "$TEMP?")]:
            tk.Button(qf, text=label, bg="#2a2a4a", fg="#cccccc", font=("微软雅黑", 9),
                      relief=tk.FLAT, padx=6, command=lambda c=cmd: self._send_cmd(c)).pack(
                      side=tk.LEFT, padx=1, fill=tk.X, expand=True)

        # 自定义命令
        self._section_label(right_frame, "💻 自定义命令")
        cmd_frame = tk.Frame(right_frame, bg="#14142a")
        cmd_frame.pack(fill=tk.X, pady=2)
        self.cmd_entry = ttk.Entry(cmd_frame, font=("Consolas", 10))
        self.cmd_entry.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        self.cmd_entry.bind("<Return>", lambda e: self._send_custom())
        tk.Button(cmd_frame, text="发送", bg="#333355", fg="white", font=("微软雅黑", 9),
                  relief=tk.FLAT, padx=8, command=self._send_custom).pack(side=tk.LEFT)

        # 底部状态栏
        self.status_bar = tk.Label(self.root, text="就绪 — 请选择串口并连接", bg="#0a0a15", fg="#666688",
                                   font=("Consolas", 9), anchor="w", padx=8)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _section_label(self, parent, text):
        tk.Label(parent, text=text, bg="#14142a", fg="#8888aa",
                 font=("微软雅黑", 9, "bold")).pack(anchor="w", pady=(8, 0))

    def _pid_row(self, parent, label, default):
        row = tk.Frame(parent, bg="#14142a")
        row.pack(fill=tk.X, pady=1)
        tk.Label(row, text=label, bg="#14142a", fg="#aaaaaa",
                 font=("微软雅黑", 9), width=12, anchor="e").pack(side=tk.LEFT, padx=2)
        entry = ttk.Entry(row, width=10, font=("Consolas", 10))
        entry.insert(0, default)
        entry.pack(side=tk.LEFT, padx=2)
        return entry

    # =================== 串口操作 ===================
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.set(ports[0])
        self._log(f"检测到串口: {', '.join(ports) if ports else '无'}")

    def _toggle_connect(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.get()
        baud = int(self.baud_combo.get())
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.connected = True
            self.connect_btn.config(state=tk.DISABLED, bg="#444444")
            self.disconnect_btn.config(state=tk.NORMAL, bg="#cc3333")
            self.status_bar.config(text=f"已连接 {port} @ {baud}", fg="#00aa44")

            # 启动后台读取线程
            self.serial_thread = SerialThread(self.ser, self._on_serial_data)
            self.serial_thread.start()

            self._log(f"✅ 已连接 {port} @ {baud}")
            # 查询当前状态
            self.root.after(500, lambda: self._send_cmd("$STATUS"))
        except Exception as e:
            messagebox.showerror("连接失败", str(e))

    def _disconnect(self):
        if self.serial_thread:
            self.serial_thread.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.ser = None
        self.connect_btn.config(state=tk.NORMAL, bg="#006622")
        self.disconnect_btn.config(state=tk.DISABLED, bg="#444444")
        self.status_bar.config(text="已断开", fg="#cc4444")
        self._log("⏹ 已断开连接")

    def _send_cmd(self, cmd):
        if not self.connected or not self.ser:
            self._log(f"⚠ 未连接, 无法发送: {cmd}")
            return
        try:
            full_cmd = cmd if cmd.endswith("\n") else cmd + "\n"
            self.ser.write(full_cmd.encode("utf-8"))
            self._log(f"→ {cmd}")
        except Exception as e:
            self._log(f"❌ 发送失败: {e}")

    def _send_custom(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self._send_cmd(cmd)
            self.cmd_entry.delete(0, tk.END)

    # =================== 数据接收与解析 ===================
    def _on_serial_data(self, data):
        """由后台线程回调, 将数据放入主线程队列"""
        if data is None:
            # 串口断开
            self.root.after(0, self._disconnect)
            return
        self.root.after(0, self._process_rx, data)

    def _process_rx(self, data):
        """主线程处理接收到的数据"""
        try:
            text = data.decode("utf-8", errors="replace")
        except:
            text = data.decode("gbk", errors="replace")

        self._log_raw(text)

        # 尝试解析 $RPT 数据
        for line in text.split("\n"):
            line = line.strip()
            if line.startswith("$RPT "):
                self._parse_rpt(line)
            elif line.startswith("$STATUS "):
                self._parse_status(line)
            elif line.startswith("$TEMP ") and "OK" in line:
                self._log(f"✅ 温度已设定")
            elif line.startswith("$PID ") and "OK" in line:
                self._log(f"✅ PID 参数已更新")
            elif line.startswith("$MODE "):
                self._log(f"✅ {line}")
            elif line.startswith("$BOOT "):
                self._log(f"🟢 {line}")

    def _parse_rpt(self, line):
        """解析 $RPT ADC=1486  T=25.1  SET=30.0  PWR=65  H=1"""
        m_temp = re.search(r"T=([\d.]+)", line)
        m_set  = re.search(r"SET=([\d.]+)", line)
        m_pwr  = re.search(r"PWR=(\d+)", line)
        m_heat = re.search(r"H=(\d)", line)
        m_adc  = re.search(r"ADC=(\d+)", line)

        if m_temp:
            self.current_temp = float(m_temp.group(1))
            self.temp_label.config(text=f"{self.current_temp:.1f} °C")
            if self.current_temp < 10:  self.temp_label.config(fg="#4488ff")
            elif self.current_temp < 30: self.temp_label.config(fg="#00ff88")
            elif self.current_temp < 50: self.temp_label.config(fg="#ffaa00")
            else: self.temp_label.config(fg="#ff4444")

        if m_set:
            self.current_setpoint = float(m_set.group(1))
            self.setpoint_label.config(text=f"目标: {self.current_setpoint:.1f} °C")

        if m_pwr:
            self.current_power = int(m_pwr.group(1))
            self.power_label.config(text=f"功率: {self.current_power}%")
            if self.current_power > 80:  self.power_label.config(fg="#ff4444")
            elif self.current_power > 30: self.power_label.config(fg="#ffaa00")
            else: self.power_label.config(fg="#888888")

        if m_heat:
            self.heater_on = (m_heat.group(1) == "1")
            self.heater_label.config(
                text=f"加热: {'🔥 ON' if self.heater_on else '❄ OFF'}",
                fg="#ff6644" if self.heater_on else "#888888")

        # 更新图表
        if m_temp and m_set:
            self.chart.add_point(self.current_temp, self.current_setpoint)

    def _parse_status(self, line):
        """解析 $STATUS 响应, 更新 UI"""
        self._parse_rpt(line)  # 复用解析逻辑
        m_mode = re.search(r"MODE=(\w+)", line)
        if m_mode:
            self.mode = m_mode.group(1)
            self.mode_label.config(text=f"模式: {self.mode}")
            if self.mode == "AUTO":
                self.auto_btn.config(bg="#004488", fg="white")
                self.manual_btn.config(bg="#444444", fg="#aaaaaa")
            else:
                self.auto_btn.config(bg="#444444", fg="#aaaaaa")
                self.manual_btn.config(bg="#886600", fg="white")

    # =================== 按钮回调 ===================
    def _set_temp(self):
        try:
            val = float(self.setpoint_entry.get())
            self._send_cmd(f"$TEMP={val:.1f}")
        except ValueError:
            messagebox.showwarning("提示", "请输入有效温度值")

    def _set_pid(self):
        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            self._send_cmd(f"$PID={kp:.2f},{ki:.3f},{kd:.2f}")
        except ValueError:
            messagebox.showwarning("提示", "请输入有效 PID 参数")

    def _set_auto(self):
        self._send_cmd("$MODE=AUTO")

    def _set_manual(self):
        self._send_cmd("$MODE=MANUAL")

    # =================== 日志 ===================
    def _log(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.log_area.insert(tk.END, f"[{timestamp}] {msg}\n")
        self.log_area.see(tk.END)

    def _log_raw(self, text):
        """原始数据不加入日志区 (避免刷屏), 仅 $RPT 等以解析后的格式显示"""
        pass  # 若需要调试原始数据, 取消下面注释:
        # for line in text.strip().split("\n"):
        #     if line.strip(): self._log(f"← {line.strip()}")

# ====================== 入口 ======================
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)

    # 定时刷新串口列表 + UI 状态
    def periodic_refresh():
        # 如果未连接, 每 2 秒刷新一次串口列表
        if not app.connected:
            if int(time.time()) % 3 == 0:
                app._refresh_ports()
        root.after(3000, periodic_refresh)

    root.after(1000, periodic_refresh)
    root.mainloop()
