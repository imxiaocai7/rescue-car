"""Motion commissioning: default reads status; enable/pulse/keyboard are explicit."""
import argparse
import ipaddress
import json
import queue
import threading
import time
from dataclasses import asdict
from motion import Client, MAX_DUTY, Op, pulse


class KeyboardState:
    """UI policy shared with tests; focus loss clears held keys and enable intent."""
    def __init__(self):
        self.keys = set()
        self.enabled = False
        self.last_ui_poll = time.monotonic()

    def stop(self):
        self.keys.clear()
        self.enabled = False

    def target(self, level):
        if not self.enabled:
            return 0, 0
        aliases = {"up": "w", "down": "s", "left": "a", "right": "d",
                   "mouse:w": "w", "mouse:s": "s", "mouse:a": "a", "mouse:d": "d"}
        keys = {aliases.get(name, name) for name in self.keys}
        if "w" in keys and "s" in keys or "a" in keys and "d" in keys:
            return 0, 0
        forward = int("w" in keys) - int("s" in keys)
        turn = int("d" in keys) - int("a" in keys)
        return max(-level, min(level, (forward + turn) * level)), max(-level, min(level, (forward - turn) * level))

    def expire_ui(self, now):
        if self.enabled and now - self.last_ui_poll >= 0.2:
            self.stop()
            return True
        return False


def keyboard(args):
    import tkinter as tk
    root = tk.Tk()
    root.title("小车电脑控制")
    root.geometry("640x500")
    root.minsize(600, 480)
    status = tk.StringVar(value="未连接；先架空车轮。连接后仍需点击使能。")
    outputs = tk.StringVar(value="STM32 回报占空比：尚无数据")
    tk.Label(root, text="双轮小车控制", font=("Microsoft YaHei UI", 18, "bold")).pack(pady=(16, 8))
    tk.Label(root, textvariable=status, wraplength=590, justify="left").pack(padx=15, pady=8)
    tk.Label(root, textvariable=outputs, font=("Microsoft YaHei UI", 11)).pack(pady=4)
    tk.Label(root, text="按住方向按钮、W/A/S/D 或方向键运行；松开后目标归零。\n空格、Esc、窗口失焦和关闭窗口会停车并禁用。", justify="center").pack(pady=8)
    lock = threading.Lock()
    state = KeyboardState()
    events = queue.Queue()
    ending = threading.Event()
    worker_thread = None
    want_arm = False
    want_stop = False

    def stop(_event=None):
        nonlocal want_stop, want_arm
        with lock:
            state.stop()
            want_stop, want_arm = True, False

    def arm():
        nonlocal want_arm
        with lock:
            if worker_thread is not None and worker_thread.is_alive():
                state.keys.clear()
                want_arm = True

    def worker():
        nonlocal want_arm, want_stop
        peer = None
        try:
            peer = Client(args.host, args.port)
            reply = peer.hello()
            events.put(("reply", reply))
            events.put(("connected", True))
            events.put(("status", "STM32 已确认连接，电机禁用；点击使能后才能运行。"))
            while not ending.is_set():
                cycle = time.monotonic()
                with lock:
                    if state.expire_ui(cycle):
                        want_stop, want_arm = True, False
                    stopping, arming = want_stop, want_arm
                    want_stop = want_arm = False
                    left, right = state.target(args.duty)
                if stopping:
                    peer.stop()
                    reply = peer.hello()
                    events.put(("status", "STM32 已确认禁用；需要重新点击使能。"))
                elif arming:
                    reply = peer.arm()
                    with lock:
                        # A concurrent focus-loss/STOP always wins over ARM completion.
                        state.enabled = not want_stop and time.monotonic() - state.last_ui_poll < 0.2
                        if not state.enabled:
                            want_stop = True
                    events.put(("status", "STM32 已确认使能；按住方向按钮或方向键运行。"))
                elif peer.armed:
                    reply = peer.request(Op.DRIVE, left, right)
                else:
                    reply = peer.request(Op.STATUS)
                events.put(("reply", reply))
                ending.wait(max(0, 0.05 - (time.monotonic() - cycle)))
        except (OSError, RuntimeError, ValueError) as exc:
            events.put(("status", f"连接已结束：{exc}；停车确认不可用，检查后手动重新连接。"))
        finally:
            with lock:
                state.stop()
                want_arm = False
            if peer is not None:
                peer.close()
            events.put(("connected", False))

    def connect():
        nonlocal worker_thread, want_arm, want_stop
        if worker_thread is not None and worker_thread.is_alive():
            return
        with lock:
            state.stop()
            want_arm = want_stop = False
        worker_thread = threading.Thread(target=worker, daemon=True)
        worker_thread.start()

    def key(event, down):
        name = event.keysym.lower()
        if name in ("space", "escape"):
            stop()
            return "break"
        elif name in ("w", "a", "s", "d", "up", "down", "left", "right"):
            with lock:
                if down and state.enabled:
                    state.keys.add(name)
                else:
                    state.keys.discard(name)
            return "break"

    def mouse(name, down):
        with lock:
            if down and state.enabled:
                state.keys.add("mouse:" + name)
            else:
                state.keys.discard("mouse:" + name)

    def close():
        stop()
        ending.set()
        root.destroy()

    def poll():
        with lock:
            state.last_ui_poll = time.monotonic()
        try:
            while True:
                kind, value = events.get_nowait()
                if kind == "status":
                    status.set(value)
                elif kind == "connected":
                    connect_button.config(state="disabled" if value else "normal")
                    arm_button.config(state="normal" if value else "disabled")
                    if not value:
                        outputs.set("STM32 回报已中断；不能确认当前物理状态")
                elif kind == "reply":
                    outputs.set(f"STM32 回报占空比：左 {value.left / 10:+g}%   右 {value.right / 10:+g}%"
                                f"   {'已使能' if value.flags & 1 else '已禁用'}")
                    arm_button.config(state="disabled" if value.flags & 1 else "normal")
        except queue.Empty:
            pass
        root.after(50, poll)

    row = tk.Frame(root)
    row.pack(pady=10)
    connect_button = tk.Button(row, text="连接小车", command=connect, width=15)
    connect_button.pack(side="left", padx=8)
    arm_button = tk.Button(row, text="使能控制", command=arm, state="disabled", width=15)
    arm_button.pack(side="left", padx=8)
    directions = tk.Frame(root)
    directions.pack(pady=8)
    for text, name, r, c in (("↑ 前进  W", "w", 0, 1), ("← 左转  A", "a", 1, 0),
                             ("右转  D →", "d", 1, 2), ("↓ 后退  S", "s", 2, 1)):
        button = tk.Button(directions, text=text, width=14, height=2)
        button.grid(row=r, column=c, padx=5, pady=3)
        button.bind("<ButtonPress-1>", lambda event, name=name: mouse(name, True))
        button.bind("<ButtonRelease-1>", lambda event, name=name: mouse(name, False))
        button.bind("<Leave>", lambda event, name=name: mouse(name, False))
    tk.Button(directions, text="停车 / 禁用", command=stop, bg="#b42318", fg="white",
              width=14, height=2).grid(row=1, column=1, padx=5, pady=3)
    tk.Label(root, text=f"目标占空比：{args.duty / 10:g}%    松开方向归零，停车后须重新使能").pack(pady=8)
    root.bind("<KeyPress>", lambda event: key(event, True))
    root.bind("<KeyRelease>", lambda event: key(event, False))
    root.bind("<FocusOut>", lambda event: root.after_idle(lambda: stop() if root.focus_displayof() is None else None))
    root.protocol("WM_DELETE_WINDOW", close)
    poll()
    root.mainloop()
    if worker_thread is not None:
        worker_thread.join(timeout=0.6)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=6002)
    parser.add_argument("--allow-hardware", action="store_true")
    parser.add_argument("--allow-motion", action="store_true")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--pulse", action="store_true")
    modes.add_argument("--keyboard", action="store_true")
    parser.add_argument("--left", type=int, default=0)
    parser.add_argument("--right", type=int, default=0)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--duty", type=int, default=100)
    args = parser.parse_args(argv)
    try:
        address = ipaddress.IPv4Address(args.host)
    except ipaddress.AddressValueError:
        parser.error("host must be numeric IPv4")
    if not address.is_loopback and not args.allow_hardware:
        parser.error("hardware connection requires --allow-hardware")
    if (args.pulse or args.keyboard) and not args.allow_motion:
        parser.error("movement requires --allow-motion")
    if not 1 <= args.port <= 65535 or not 1 <= args.duty <= MAX_DUTY:
        parser.error("invalid port/duty")
    if not -MAX_DUTY <= args.left <= MAX_DUTY or not -MAX_DUTY <= args.right <= MAX_DUTY:
        parser.error("pulse duty outside -250..250")
    if not 0 < args.duration <= 1:
        parser.error("pulse duration must be >0 and <=1 second")
    return args


def main():
    args = parse_args()
    if args.keyboard:
        keyboard(args)
        return
    peer = None
    try:
        peer = Client(args.host, args.port)
        reply = peer.hello()
        if args.pulse:
            pulse(peer, args.left, args.right, args.duration)
            reply = peer.hello()
        print(json.dumps({"reply": asdict(reply), "hardware_verified": False}, indent=2))
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(f"Motion commissioning FAILED: {exc}") from None
    finally:
        if peer is not None:
            peer.close()


if __name__ == "__main__":
    main()
