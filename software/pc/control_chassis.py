"""Windows/Tk manual chassis panel; defaults to an embedded localhost simulator."""

import argparse
import ipaddress
import time
import tkinter as tk
from tkinter import ttk

from chassis_simulator import ChassisSimulatorServer
from manual_chassis import ControlState, ManualChassisController, ManualMotionWorker
from rover_telemetry import TelemetryStatus


class ManualControlApp:
    def __init__(
        self, root: tk.Tk, host: str, port: int, *, demo: bool,
        speed_mm_s: int = 80, wheel_diameter_mm: float | None = None,
        simulator: ChassisSimulatorServer | None = None, autoconnect: bool = True,
    ):
        self.root = root
        self.simulator = simulator
        self.controller = ManualChassisController(speed_mm_s, wheel_diameter_mm=wheel_diameter_mm)
        self.worker = ManualMotionWorker(self.controller, host, port)
        self._closing = False
        self._closed = False
        self._mouse_key: str | None = None
        self._refresh_job = None
        self._focus_job = None
        self._connect_job = None
        root.title("底盘手动控制 — " + ("本地模拟" if demo else "实车接口 / 未验收"))
        root.minsize(720, 570)
        root.protocol("WM_DELETE_WINDOW", self.close)
        panel = ttk.Frame(root, padding=18)
        panel.pack(fill="both", expand=True)
        panel.columnconfigure(0, weight=1)
        banner = (
            "本地模拟模式：不会连接板子。轮径用于合成数据，不是实车标定。"
            if demo else "实车接口：必须先完成固件、电流、方向与断线保护验收；软件停止不是硬件急停。"
        )
        ttk.Label(panel, text=banner, wraplength=680, foreground="#984600").grid(row=0, sticky="w")
        ttk.Label(panel, text=f"运动端点：{host}:{port}  |  目标单位：mm/s  |  租约：300ms").grid(
            row=1, sticky="w", pady=(8, 12),
        )
        actions = ttk.Frame(panel)
        actions.grid(row=2, sticky="ew")
        self.connect_button = ttk.Button(actions, text="连接 / 重新连接", command=self.connect)
        self.connect_button.pack(side="left", padx=(0, 8))
        self.disconnect_button = ttk.Button(actions, text="断开", command=self.disconnect)
        self.disconnect_button.pack(side="left", padx=(0, 8))
        self.arm_button = ttk.Button(actions, text="允许运动", command=self.arm)
        self.arm_button.pack(side="left", padx=(0, 8))
        self.stop_button = tk.Button(
            actions, text="停止并锁定  [空格 / Esc]", background="#aa2222", foreground="white",
            command=self.stop, padx=10, pady=5,
        )
        self.stop_button.pack(side="left")
        self.state_text = tk.StringVar()
        self.reason_text = tk.StringVar()
        ttk.Label(panel, textvariable=self.state_text, font=("Microsoft YaHei UI", 13)).grid(
            row=3, sticky="w", pady=(12, 2),
        )
        ttk.Label(panel, textvariable=self.reason_text, wraplength=680).grid(row=4, sticky="w")
        speed_panel = ttk.Frame(panel)
        speed_panel.grid(row=5, sticky="ew", pady=12)
        self.speed_text = tk.StringVar(value=f"速度：{speed_mm_s} mm/s")
        ttk.Label(speed_panel, textvariable=self.speed_text).pack(side="left")
        self.speed_scale = tk.Scale(
            speed_panel, from_=1, to=300, orient="horizontal", length=410,
            command=self._change_speed, showvalue=False,
        )
        self.speed_scale.set(speed_mm_s)
        self.speed_scale.pack(side="left", padx=12)
        ttk.Label(panel, text="按住运行，松开停止。W+A / W+D 可弧线转弯；相反按键同时按下时归零。").grid(
            row=6, sticky="w",
        )
        directions = ttk.Frame(panel)
        directions.grid(row=7, pady=12)
        self.direction_buttons = []
        for key, label, row, column in (
            ("w", "前进 W", 0, 1), ("a", "原地左转 A", 1, 0),
            ("s", "后退 S", 1, 1), ("d", "原地右转 D", 1, 2),
        ):
            button = ttk.Button(directions, text=label, width=16)
            button.grid(row=row, column=column, padx=5, pady=4)
            button.bind("<ButtonPress-1>", lambda event, key=key: self._mouse_press(key))
            self.direction_buttons.append(button)
        feedback = ttk.LabelFrame(panel, text="双轮状态（正数表示车体前进方向）", padding=10)
        feedback.grid(row=8, sticky="ew")
        self.targets_text = tk.StringVar()
        self.sent_text = tk.StringVar()
        self.measured_text = tk.StringVar()
        self.counts_text = tk.StringVar()
        self.link_text = tk.StringVar()
        for row, variable in enumerate((
            self.targets_text, self.sent_text, self.measured_text, self.counts_text, self.link_text,
        )):
            ttk.Label(feedback, textvariable=variable).grid(row=row, sticky="w", pady=2)
        ttk.Label(
            panel, text="默认 80mm/s 和 300mm/s 上限只是软件初始配置，不代表实车安全速度。",
            foreground="#666666", wraplength=680,
        ).grid(row=9, sticky="w", pady=(10, 0))
        root.bind("<KeyPress>", self._key_press)
        root.bind("<KeyRelease>", self._key_release)
        root.bind("<ButtonRelease-1>", self._mouse_release)
        root.bind("<FocusOut>", self._focus_out)
        self._refresh()
        if autoconnect:
            self._connect_job = root.after(0, self.connect)

    def connect(self) -> None:
        if self._closing or self.worker.running:
            return
        self.worker.start()
        self.root.focus_set()
        self._refresh_now()

    def disconnect(self) -> None:
        self.worker.request_stop()
        self._refresh_now()

    def arm(self) -> None:
        with self.worker.lock:
            self.controller.arm(time.monotonic())
        self.root.focus_set()
        self._refresh_now()

    def stop(self) -> None:
        with self.worker.lock:
            self.controller.disarm("操作者停止")
        self._refresh_now()

    def _change_speed(self, value: str) -> None:
        speed = int(float(value))
        with self.worker.lock:
            self.controller.set_speed(speed)
        self.speed_text.set(f"速度：{speed} mm/s")

    def _key_press(self, event):
        key = event.keysym.lower()
        if key in ("space", "escape"):
            self.stop()
            return "break"
        if key in ("w", "a", "s", "d"):
            with self.worker.lock:
                self.controller.press(key)
            return "break"

    def _key_release(self, event):
        if event.keysym.lower() in ("w", "a", "s", "d"):
            with self.worker.lock:
                self.controller.release(event.keysym)
            return "break"

    def _mouse_press(self, key: str):
        self.root.focus_set()
        with self.worker.lock:
            if self._mouse_key is not None:
                self.controller.release(self._mouse_key)
            self._mouse_key = key
            self.controller.press(key)
        return "break"

    def _mouse_release(self, _event):
        with self.worker.lock:
            if self._mouse_key is not None:
                self.controller.release(self._mouse_key)
                self._mouse_key = None

    def _focus_out(self, _event):
        # Moving focus between this window's widgets is not losing app focus.
        if self._focus_job is not None:
            self.root.after_cancel(self._focus_job)
        self._focus_job = self.root.after(10, self._check_focus)

    def _check_focus(self):
        if not self._closing and self.root.focus_displayof() is None:
            with self.worker.lock:
                self.controller.disarm("窗口失去焦点")
            self._refresh_now()

    def _refresh_now(self):
        if self._closing:
            return
        if self._refresh_job is not None:
            self.root.after_cancel(self._refresh_job)
        self._refresh()

    def _refresh(self):
        if self._closing:
            return
        now_s = time.monotonic()
        with self.worker.lock:
            self.controller.input_alive(now_s)
            status = self.controller.status(now_s)
        self.state_text.set(status.state.value)
        self.reason_text.set(status.reason)
        self.targets_text.set(f"目标轮速：左 {status.requested_speeds[0]}  /  右 {status.requested_speeds[1]} mm/s")
        self.sent_text.set(f"最近发送：左 {status.sent_speeds[0]}  /  右 {status.sent_speeds[1]} mm/s（不等于执行成功）")
        if status.measured_speeds is None:
            self.measured_text.set("编码器估算：--（未填写实测轮径，或尚无连续有效样本）")
        else:
            left, right = status.measured_speeds
            self.measured_text.set(f"编码器估算：左 {left:.1f}  /  右 {right:.1f} mm/s")
        frame = status.telemetry
        if frame is None:
            self.counts_text.set("编码器累计：--")
            self.link_text.set("反馈：尚未收到 STM32 遥测")
        else:
            self.counts_text.set(f"编码器累计：左 {frame.left_encoder_total}  /  右 {frame.right_encoder_total}")
            ack = frame.last_command_sequence if frame.status & TelemetryStatus.LAST_COMMAND_VALID else "--"
            age_ms = status.telemetry_age_s * 1000
            self.link_text.set(f"反馈年龄：{age_ms:.0f}ms  |  已接收序号：{ack}  |  待确认：{status.pending_sequence}")
        self.connect_button.configure(state="disabled" if self.worker.running else "normal")
        self.disconnect_button.configure(state="normal" if self.worker.running else "disabled")
        self.arm_button.configure(state="normal" if status.can_arm else "disabled")
        for button in self.direction_buttons:
            button.configure(state="normal" if status.state == ControlState.ARMED else "disabled")
        self._refresh_job = self.root.after(50, self._refresh)

    def close(self):
        if self._closing:
            return
        self._closing = True
        for job in (self._refresh_job, self._focus_job, self._connect_job):
            if job is not None:
                self.root.after_cancel(job)
        self.worker.request_stop()
        self._wait_close()

    def _wait_close(self):
        if self.worker.running:
            self.root.after(20, self._wait_close)
            return
        if self.simulator is not None:
            self.simulator.close()
        self._closed = True
        self.root.destroy()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Manual chassis panel; no arguments means local simulation")
    destination = parser.add_mutually_exclusive_group()
    destination.add_argument("--demo", action="store_true", help="Embedded localhost simulator (default)")
    destination.add_argument("--host", help="Numeric IPv4 address of an external simulator or ESP32")
    parser.add_argument("--port", type=int, help="External TCP port: default 5001; independent simulator uses 5501")
    parser.add_argument("--allow-hardware", action="store_true", help="Explicitly allow a non-loopback endpoint; NOT safety approval")
    parser.add_argument("--speed-mm-s", type=int, default=80)
    parser.add_argument("--wheel-diameter-mm", type=float, help="Actual measured diameter; only simulation has a default")
    args = parser.parse_args(argv)
    if args.host is not None:
        try:
            address = ipaddress.IPv4Address(args.host)
        except ipaddress.AddressValueError:
            parser.error("--host must be a numeric IPv4 address")
        if not address.is_loopback and not args.allow_hardware:
            parser.error("non-loopback targets require --allow-hardware; hardware commissioning is still required")
    elif args.port is not None or args.allow_hardware:
        parser.error("--port / --allow-hardware require --host")
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    try:
        ManualChassisController(args.speed_mm_s, wheel_diameter_mm=args.wheel_diameter_mm)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def main() -> None:
    args = parse_args()
    demo = args.host is None
    diameter = args.wheel_diameter_mm
    simulator = None
    if demo:
        diameter = diameter if diameter is not None else 75.0
        simulator = ChassisSimulatorServer(wheel_diameter_mm=diameter).start()
        host, port = simulator.host, simulator.port
    else:
        host, port = args.host, args.port or 5001
    try:
        root = tk.Tk()
        app = ManualControlApp(
            root, host, port, demo=demo, speed_mm_s=args.speed_mm_s,
            wheel_diameter_mm=diameter, simulator=simulator, autoconnect=demo,
        )
        try:
            root.mainloop()
        finally:
            app.worker.close()
    finally:
        if simulator is not None:
            simulator.close()


if __name__ == "__main__":
    main()
