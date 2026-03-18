from __future__ import annotations
from __future__ import annotations

import argparse
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext, ttk

import serial
from serial.tools import list_ports


FRAME_DOWN = 0xFE
FRAME_UP = 0xFD

CMD_HELLO = 0xA0
CMD_INFO = 0xA1
CMD_STA = 0xA2
CMD_MOT = 0xB1
CMD_TEMP = 0xB2

MAX_RPM = 2000
MAX_TEMP = 300


def int16_to_bytes(value: int) -> tuple[int, int]:
    high = (value >> 8) & 0xFF
    low = value & 0xFF
    return high, low


def bytes_to_int16(high: int, low: int) -> int:
    return (high << 8) | low


class Led485AsciiDisplay:
    """LED-485 ASCII protocol driver: $AAA,text#"""

    def __init__(
        self,
        port: str,
        address: int = 1,
        baudrate: int = 9600,
        timeout: float = 0.2,
    ) -> None:
        if not 1 <= address <= 254:
            raise ValueError("address must be in range 1..254")
        self.address = address
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def send_ascii(self, text: str) -> None:
        frame = f"${self.address:03d},{text}#"
        self.serial.write(frame.encode("ascii", errors="ignore"))


class DalongMSController:
    """Dalong MS controller with robust checksum and frame parsing."""

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 0.2) -> None:
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout,
        )
        self.lock = threading.Lock()
        self.checksum_mode = "exclude_header"
        if not self.ser.is_open:
            self.ser.open()
        time.sleep(0.15)

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    @staticmethod
    def _validate_checksum(frame: bytes) -> bool:
        if len(frame) < 2:
            return False
        expected_exclude = sum(frame[1:-1]) & 0xFF
        expected_include = sum(frame[:-1]) & 0xFF
        return frame[-1] in (expected_exclude, expected_include)

    @staticmethod
    def _calc_checksum_for_mode(frame_without_checksum: list[int], mode: str) -> int:
        if mode == "include_header":
            return sum(frame_without_checksum) & 0xFF
        return sum(frame_without_checksum[1:]) & 0xFF

    @staticmethod
    def _find_response_frame(raw: bytes, cmd: int | list[int], min_len: int, max_len: int | None = None) -> bytes:
        cmd_list = [cmd] if isinstance(cmd, int) else list(cmd)
        for i in range(0, len(raw) - 1):
            if raw[i] != FRAME_UP or raw[i + 1] not in cmd_list:
                continue
            available = len(raw) - i
            if available < min_len:
                continue

            search_max = available if max_len is None else min(available, max_len)
            for frame_len in range(search_max, min_len - 1, -1):
                candidate = raw[i : i + frame_len]
                if DalongMSController._validate_checksum(candidate):
                    return candidate
        return b""

    def _read_raw_with_deadline(self, total_timeout: float = 0.4) -> bytes:
        deadline = time.time() + total_timeout
        chunks = bytearray()
        while time.time() < deadline:
            waiting = self.ser.in_waiting
            if waiting > 0:
                chunks.extend(self.ser.read(waiting))
            else:
                chunks.extend(self.ser.read(1))

            if chunks:
                time.sleep(0.01)
        return bytes(chunks)

    def send_cmd(
        self,
        cmd: int,
        param1: int = 0,
        param2: int = 0,
        param3: int = 0,
        expected_len: int = 6,
        total_timeout: float = 0.4,
        expected_cmd: int | list[int] | None = None,
        max_len: int | None = None,
    ) -> bytes:
        frame = [FRAME_DOWN, cmd, param1, param2, param3]
        checksum = self._calc_checksum_for_mode(frame, self.checksum_mode)
        frame.append(checksum)

        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(bytes(frame))
            self.ser.flush()
            raw = self._read_raw_with_deadline(total_timeout=total_timeout)

        response_cmd = cmd if expected_cmd is None else expected_cmd
        parsed = self._find_response_frame(raw, response_cmd, expected_len, max_len=max_len)

        if parsed:
            return parsed

        if self.checksum_mode == "exclude_header":
            alt_mode = "include_header"
        else:
            alt_mode = "exclude_header"

        frame_alt = [FRAME_DOWN, cmd, param1, param2, param3]
        frame_alt.append(self._calc_checksum_for_mode(frame_alt, alt_mode))

        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(bytes(frame_alt))
            self.ser.flush()
            raw_alt = self._read_raw_with_deadline(total_timeout=total_timeout)

        parsed_alt = self._find_response_frame(raw_alt, response_cmd, expected_len, max_len=max_len)
        if parsed_alt:
            self.checksum_mode = alt_mode
            return parsed_alt

        return b""

    def hello(self) -> bool:
        resp = self.send_cmd(CMD_HELLO, 0, 0, 0, expected_len=6, total_timeout=0.8)
        return len(resp) >= 6 and resp[2] == 0

    def set_stir(self, rpm: int) -> bool:
        high, low = int16_to_bytes(rpm)
        resp = self.send_cmd(CMD_MOT, high, low, 0, expected_len=6)
        return len(resp) >= 6 and resp[2] == 0

    def set_heat(self, temp: int) -> bool:
        high, low = int16_to_bytes(temp)
        resp = self.send_cmd(CMD_TEMP, high, low, 0, expected_len=6)
        return len(resp) >= 6 and resp[2] == 0

    def get_status(self) -> dict[str, int] | None:
        resp = self.send_cmd(
            CMD_STA,
            0,
            0,
            0,
            expected_len=11,
            expected_cmd=[CMD_STA, CMD_INFO],
            max_len=15,
            total_timeout=0.5,
        )
        if len(resp) >= 11:
            return {
                "set_rpm": bytes_to_int16(resp[2], resp[3]),
                "cur_rpm": bytes_to_int16(resp[4], resp[5]),
                "set_temp": bytes_to_int16(resp[6], resp[7]),
                "cur_temp": bytes_to_int16(resp[8], resp[9]),
            }
        return None


@dataclass
class WorkerConfig:
    port: str
    address: int
    baudrate: int
    digits: int
    mode: str
    countdown_start: int
    switch_seconds: float
    stir_port: str = ""
    stir_baudrate: int = 9600
    poll_seconds: float = 0.5


def fit_text(text: str, digits: int) -> str:
    if len(text) > digits:
        return text[-digits:]
    return text.rjust(digits, " ")


def is_stopped(stop_event: threading.Event | None) -> bool:
    return stop_event is not None and stop_event.is_set()


def sleep_or_stop(seconds: float, stop_event: threading.Event | None, tick: float = 0.05) -> bool:
    if stop_event is None:
        time.sleep(seconds)
        return False

    end_at = time.monotonic() + seconds
    while time.monotonic() < end_at:
        if stop_event.is_set():
            return True
        time.sleep(min(tick, end_at - time.monotonic()))
    return stop_event.is_set()


def run_clock(display: Led485AsciiDisplay, digits: int, switch_seconds: float, stop_event: threading.Event | None) -> None:
    showing_date = True
    next_switch = time.monotonic()
    last_sent = ""

    while not is_stopped(stop_event):
        now = time.localtime()
        date_text = f"{now.tm_mon:02d}-{now.tm_mday:02d}"
        time_text = f"{now.tm_hour:02d}.{now.tm_min:02d}"
        payload = fit_text(date_text if showing_date else time_text, digits)

        if payload != last_sent:
            display.send_ascii(payload)
            last_sent = payload

        t = time.monotonic()
        if t >= next_switch:
            showing_date = not showing_date
            next_switch = t + switch_seconds
            last_sent = ""

        if sleep_or_stop(0.1, stop_event):
            return


def run_countdown(display: Led485AsciiDisplay, digits: int, start_seconds: int, stop_event: threading.Event | None) -> None:
    remaining = start_seconds
    while remaining >= 0 and not is_stopped(stop_event):
        display.send_ascii(fit_text(str(remaining), digits))
        if sleep_or_stop(1.0, stop_event):
            return
        remaining -= 1

    eof_text = fit_text("EOF", digits)
    blank = " " * digits
    while not is_stopped(stop_event):
        display.send_ascii(eof_text)
        if sleep_or_stop(0.5, stop_event):
            return
        display.send_ascii(blank)
        if sleep_or_stop(0.5, stop_event):
            return


def format_stirrer_payload(cur_temp: int, cur_rpm: int, digits: int, showing_temp: bool) -> str:
    # Keep prefix at the first digit and right-align numbers to maximize readability.
    # Speed prefix uses '5' to mimic 's' on seven-segment display.
    if digits <= 1:
        return "t" if showing_temp else "5"

    if showing_temp:
        number_part = fit_text(str(cur_temp), digits - 1)
        return f"t{number_part}"

    number_part = fit_text(str(cur_rpm), digits - 1)
    return f"5{number_part}"


def run_stirrer_telemetry_cli(config: WorkerConfig, stop_event: threading.Event | None = None) -> None:
    if not config.stir_port:
        raise ValueError("stir_port is required in stirrer mode")

    display = Led485AsciiDisplay(config.port, config.address, config.baudrate)
    controller = DalongMSController(config.stir_port, config.stir_baudrate)

    try:
        last_status: dict[str, int] | None = None
        last_status_ts = 0.0
        last_poll = 0.0
        showing_temp = True
        next_switch = time.monotonic()
        last_sent = ""

        while not is_stopped(stop_event):
            now_mono = time.monotonic()

            if now_mono - last_poll >= config.poll_seconds:
                last_poll = now_mono
                status = controller.get_status()
                if status is not None:
                    last_status = status
                    last_status_ts = now_mono

            if last_status is None or (now_mono - last_status_ts > 3.0):
                payload = fit_text("----", config.digits)
            else:
                payload = format_stirrer_payload(
                    last_status["cur_temp"],
                    last_status["cur_rpm"],
                    config.digits,
                    showing_temp,
                )

            if payload != last_sent:
                display.send_ascii(payload)
                last_sent = payload

            if now_mono >= next_switch:
                showing_temp = not showing_temp
                next_switch = now_mono + config.switch_seconds
                last_sent = ""

            if sleep_or_stop(0.1, stop_event):
                return
    finally:
        controller.close()
        display.close()


class App:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("LED-485-055 与搅拌器联动控制台")
        self.root.geometry("900x680")

        self.worker_thread: threading.Thread | None = None
        self.stop_event: threading.Event | None = None
        self.stirrer_cmd_q: queue.Queue[tuple[str, int]] = queue.Queue()

        self.port_var = tk.StringVar()
        self.address_var = tk.StringVar(value="1")
        self.baudrate_var = tk.StringVar(value="9600")
        self.digits_var = tk.StringVar(value="5")

        self.mode_var = tk.StringVar(value="clock")
        self.switch_seconds_var = tk.StringVar(value="2.0")
        self.countdown_start_var = tk.StringVar(value="88")

        self.stir_port_var = tk.StringVar()
        self.stir_baudrate_var = tk.StringVar(value="9600")
        self.poll_seconds_var = tk.StringVar(value="0.5")
        self.stir_set_rpm_var = tk.StringVar(value="500")
        self.stir_set_temp_var = tk.StringVar(value="50")
        self.cur_rpm_var = tk.StringVar(value="--")
        self.cur_temp_var = tk.StringVar(value="--")
        self.set_rpm_var = tk.StringVar(value="--")
        self.set_temp_var = tk.StringVar(value="--")

        self.status_var = tk.StringVar(value="未运行")

        self.btn_set_rpm: ttk.Button | None = None
        self.btn_set_temp: ttk.Button | None = None
        self.btn_stop_all: ttk.Button | None = None

        self._build_ui()
        self.refresh_ports()
        self._on_mode_changed()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        led_frame = ttk.LabelFrame(main, text="数码管参数", padding=10)
        led_frame.pack(fill="x")

        ttk.Label(led_frame, text="数码管串口").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(led_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(6, 10), sticky="w")
        ttk.Button(led_frame, text="刷新串口", command=self.refresh_ports).grid(row=0, column=2, sticky="w")

        ttk.Label(led_frame, text="地址").grid(row=0, column=3, padx=(20, 0), sticky="w")
        ttk.Entry(led_frame, textvariable=self.address_var, width=8).grid(row=0, column=4, padx=(6, 10), sticky="w")

        ttk.Label(led_frame, text="波特率").grid(row=0, column=5, sticky="w")
        ttk.Combobox(
            led_frame,
            textvariable=self.baudrate_var,
            values=["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"],
            width=10,
            state="readonly",
        ).grid(row=0, column=6, padx=(6, 10), sticky="w")

        ttk.Label(led_frame, text="位数").grid(row=0, column=7, sticky="w")
        ttk.Combobox(
            led_frame,
            textvariable=self.digits_var,
            values=["2", "3", "4", "5", "6"],
            width=8,
            state="readonly",
        ).grid(row=0, column=8, padx=(6, 0), sticky="w")

        mode_frame = ttk.LabelFrame(main, text="运行模式", padding=10)
        mode_frame.pack(fill="x", pady=(10, 0))

        ttk.Radiobutton(mode_frame, text="日期/时间", variable=self.mode_var, value="clock", command=self._on_mode_changed).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Radiobutton(mode_frame, text="倒计时", variable=self.mode_var, value="countdown", command=self._on_mode_changed).grid(
            row=0, column=1, padx=(16, 0), sticky="w"
        )
        ttk.Radiobutton(mode_frame, text="搅拌器联动", variable=self.mode_var, value="stirrer", command=self._on_mode_changed).grid(
            row=0, column=2, padx=(16, 0), sticky="w"
        )

        self.clock_frame = ttk.Frame(mode_frame)
        self.clock_frame.grid(row=1, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Label(self.clock_frame, text="切换间隔(秒)").pack(side="left")
        ttk.Entry(self.clock_frame, textvariable=self.switch_seconds_var, width=8).pack(side="left", padx=(6, 0))

        self.countdown_frame = ttk.Frame(mode_frame)
        self.countdown_frame.grid(row=2, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Label(self.countdown_frame, text="起始秒数").pack(side="left")
        ttk.Entry(self.countdown_frame, textvariable=self.countdown_start_var, width=8).pack(side="left", padx=(6, 0))

        self.stir_frame = ttk.LabelFrame(main, text="搅拌器参数与控制", padding=10)
        self.stir_frame.pack(fill="x", pady=(10, 0))

        ttk.Label(self.stir_frame, text="搅拌器串口").grid(row=0, column=0, sticky="w")
        self.stir_port_combo = ttk.Combobox(self.stir_frame, textvariable=self.stir_port_var, width=12, state="readonly")
        self.stir_port_combo.grid(row=0, column=1, padx=(6, 10), sticky="w")

        ttk.Label(self.stir_frame, text="波特率").grid(row=0, column=2, sticky="w")
        ttk.Combobox(
            self.stir_frame,
            textvariable=self.stir_baudrate_var,
            values=["9600", "19200", "38400", "57600", "115200"],
            width=10,
            state="readonly",
        ).grid(row=0, column=3, padx=(6, 10), sticky="w")

        ttk.Label(self.stir_frame, text="采样间隔(秒)").grid(row=0, column=4, sticky="w")
        ttk.Entry(self.stir_frame, textvariable=self.poll_seconds_var, width=8).grid(row=0, column=5, padx=(6, 10), sticky="w")

        ttk.Label(self.stir_frame, text="设置转速").grid(row=1, column=0, pady=(10, 0), sticky="w")
        ttk.Spinbox(self.stir_frame, from_=0, to=MAX_RPM, increment=50, textvariable=self.stir_set_rpm_var, width=10).grid(
            row=1, column=1, pady=(10, 0), sticky="w"
        )
        self.btn_set_rpm = ttk.Button(self.stir_frame, text="下发转速", command=self.send_set_rpm, state="disabled")
        self.btn_set_rpm.grid(row=1, column=2, pady=(10, 0), sticky="w")

        ttk.Label(self.stir_frame, text="设置温度").grid(row=1, column=3, pady=(10, 0), sticky="w")
        ttk.Spinbox(self.stir_frame, from_=0, to=MAX_TEMP, increment=5, textvariable=self.stir_set_temp_var, width=10).grid(
            row=1, column=4, pady=(10, 0), sticky="w"
        )
        self.btn_set_temp = ttk.Button(self.stir_frame, text="下发温度", command=self.send_set_temp, state="disabled")
        self.btn_set_temp.grid(row=1, column=5, pady=(10, 0), sticky="w")

        self.btn_stop_all = ttk.Button(self.stir_frame, text="停止搅拌+加热", command=self.send_stop_all, state="disabled")
        self.btn_stop_all.grid(row=2, column=0, columnspan=2, pady=(10, 0), sticky="w")

        status_row = ttk.Frame(self.stir_frame)
        status_row.grid(row=2, column=2, columnspan=4, pady=(10, 0), sticky="w")
        ttk.Label(status_row, text="设定转速").pack(side="left")
        ttk.Label(status_row, textvariable=self.set_rpm_var).pack(side="left", padx=(6, 12))
        ttk.Label(status_row, text="实际转速").pack(side="left")
        ttk.Label(status_row, textvariable=self.cur_rpm_var).pack(side="left", padx=(6, 12))
        ttk.Label(status_row, text="设定温度").pack(side="left")
        ttk.Label(status_row, textvariable=self.set_temp_var).pack(side="left", padx=(6, 12))
        ttk.Label(status_row, text="实际温度").pack(side="left")
        ttk.Label(status_row, textvariable=self.cur_temp_var).pack(side="left", padx=(6, 0))

        controls = ttk.Frame(main)
        controls.pack(fill="x", pady=(12, 0))
        self.start_btn = ttk.Button(controls, text="开始", command=self.start)
        self.start_btn.pack(side="left")
        self.stop_btn = ttk.Button(controls, text="停止", command=self.stop, state="disabled")
        self.stop_btn.pack(side="left", padx=(8, 0))
        ttk.Label(controls, textvariable=self.status_var).pack(side="left", padx=(16, 0))

        log_box = ttk.LabelFrame(main, text="日志", padding=8)
        log_box.pack(fill="both", expand=True, pady=(10, 0))
        self.log_text = scrolledtext.ScrolledText(log_box, height=14, state="disabled")
        self.log_text.pack(fill="both", expand=True)

    def _on_mode_changed(self) -> None:
        mode = self.mode_var.get()
        if mode == "clock":
            self.clock_frame.grid()
            self.countdown_frame.grid_remove()
            self.stir_frame.pack_forget()
        elif mode == "countdown":
            self.clock_frame.grid_remove()
            self.countdown_frame.grid()
            self.stir_frame.pack_forget()
        else:
            self.clock_frame.grid_remove()
            self.countdown_frame.grid_remove()
            self.stir_frame.pack(fill="x", pady=(10, 0))

    def refresh_ports(self) -> None:
        ports = sorted([p.device for p in list_ports.comports()])
        self.port_combo["values"] = ports
        self.stir_port_combo["values"] = ports

        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if ports and self.stir_port_var.get() not in ports:
            self.stir_port_var.set(ports[1] if len(ports) > 1 else ports[0])

        self.log(f"检测到串口: {', '.join(ports) if ports else '无'}")

    def log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {text}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def set_running(self, running: bool) -> None:
        self.start_btn.configure(state="disabled" if running else "normal")
        self.stop_btn.configure(state="normal" if running else "disabled")
        self.status_var.set("运行中" if running else "未运行")

        stir_ctrl_state = "normal" if running and self.mode_var.get() == "stirrer" else "disabled"
        if self.btn_set_rpm:
            self.btn_set_rpm.configure(state=stir_ctrl_state)
        if self.btn_set_temp:
            self.btn_set_temp.configure(state=stir_ctrl_state)
        if self.btn_stop_all:
            self.btn_stop_all.configure(state=stir_ctrl_state)

    def _build_config(self) -> WorkerConfig:
        port = self.port_var.get().strip()
        if not port:
            raise ValueError("请选择数码管串口")

        address = int(self.address_var.get().strip())
        if not 1 <= address <= 254:
            raise ValueError("地址范围必须是 1-254")

        baudrate = int(self.baudrate_var.get().strip())
        digits = int(self.digits_var.get().strip())

        mode = self.mode_var.get().strip()
        switch_seconds = float(self.switch_seconds_var.get().strip())
        if switch_seconds <= 0:
            raise ValueError("切换间隔必须大于 0")

        countdown_start = int(self.countdown_start_var.get().strip())
        if countdown_start < 0:
            raise ValueError("起始秒数必须 >= 0")

        stir_port = self.stir_port_var.get().strip()
        stir_baudrate = int(self.stir_baudrate_var.get().strip())
        poll_seconds = float(self.poll_seconds_var.get().strip())
        if poll_seconds <= 0:
            raise ValueError("采样间隔必须大于 0")

        if mode == "stirrer":
            if not stir_port:
                raise ValueError("搅拌器联动模式必须选择搅拌器串口")
            if stir_port == port:
                raise ValueError("数码管串口和搅拌器串口不能相同")

        return WorkerConfig(
            port=port,
            address=address,
            baudrate=baudrate,
            digits=digits,
            mode=mode,
            countdown_start=countdown_start,
            switch_seconds=switch_seconds,
            stir_port=stir_port,
            stir_baudrate=stir_baudrate,
            poll_seconds=poll_seconds,
        )

    def start(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            return

        try:
            config = self._build_config()
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        self.stop_event = threading.Event()
        self.set_running(True)
        self.log(
            f"启动 mode={config.mode}, led={config.port}@{config.baudrate}, addr={config.address}, digits={config.digits}, stir={config.stir_port}@{config.stir_baudrate}"
        )

        def worker() -> None:
            try:
                if config.mode == "clock":
                    display = Led485AsciiDisplay(config.port, config.address, config.baudrate)
                    try:
                        run_clock(display, config.digits, config.switch_seconds, self.stop_event)
                    finally:
                        display.close()
                elif config.mode == "countdown":
                    display = Led485AsciiDisplay(config.port, config.address, config.baudrate)
                    try:
                        run_countdown(display, config.digits, config.countdown_start, self.stop_event)
                    finally:
                        display.close()
                else:
                    self._run_stirrer_mode(config)
            except Exception as exc:  # noqa: BLE001
                self.root.after(0, lambda: self.log(f"异常: {exc}"))
                self.root.after(0, lambda: messagebox.showerror("运行异常", str(exc)))
            finally:
                self.root.after(0, lambda: self.set_running(False))
                self.root.after(0, lambda: self.log("已停止"))

        self.worker_thread = threading.Thread(target=worker, daemon=True)
        self.worker_thread.start()

    def _run_stirrer_mode(self, config: WorkerConfig) -> None:
        display = Led485AsciiDisplay(config.port, config.address, config.baudrate)
        stirrer = DalongMSController(config.stir_port, config.stir_baudrate)

        try:
            if stirrer.hello():
                self.root.after(0, lambda: self.log("搅拌器握手成功"))
            else:
                self.root.after(0, lambda: self.log("搅拌器握手失败，继续尝试读取状态"))

            last_status: dict[str, int] | None = None
            last_status_ts = 0.0
            last_poll = 0.0
            showing_temp = True
            next_switch = time.monotonic()
            last_sent = ""

            while not is_stopped(self.stop_event):
                now_mono = time.monotonic()

                while True:
                    try:
                        cmd, value = self.stirrer_cmd_q.get_nowait()
                    except queue.Empty:
                        break

                    if cmd == "set_rpm":
                        ok = stirrer.set_stir(value)
                        self.root.after(0, lambda ok=ok, value=value: self.log(f"下发转速 {value} -> {'OK' if ok else 'FAIL'}"))
                    elif cmd == "set_temp":
                        ok = stirrer.set_heat(value)
                        self.root.after(0, lambda ok=ok, value=value: self.log(f"下发温度 {value} -> {'OK' if ok else 'FAIL'}"))
                    elif cmd == "stop_all":
                        ok1 = stirrer.set_stir(0)
                        ok2 = stirrer.set_heat(0)
                        self.root.after(0, lambda ok1=ok1, ok2=ok2: self.log(f"停止命令 -> stir={'OK' if ok1 else 'FAIL'}, heat={'OK' if ok2 else 'FAIL'}"))

                if now_mono - last_poll >= config.poll_seconds:
                    last_poll = now_mono
                    try:
                        status = stirrer.get_status()
                        if status is not None:
                            last_status = status
                            last_status_ts = now_mono
                            self.root.after(0, lambda s=status: self._update_stirrer_status_ui(s))
                    except Exception as exc:  # noqa: BLE001
                        self.root.after(0, lambda exc=exc: self.log(f"读取状态失败: {exc}"))

                if last_status is None or (now_mono - last_status_ts > 3.0):
                    payload = fit_text("----", config.digits)
                else:
                    payload = format_stirrer_payload(
                        last_status["cur_temp"],
                        last_status["cur_rpm"],
                        config.digits,
                        showing_temp,
                    )

                if payload != last_sent:
                    display.send_ascii(payload)
                    last_sent = payload

                if now_mono >= next_switch:
                    showing_temp = not showing_temp
                    next_switch = now_mono + config.switch_seconds
                    last_sent = ""

                if sleep_or_stop(0.1, self.stop_event):
                    return
        finally:
            stirrer.close()
            display.close()

    def _update_stirrer_status_ui(self, status: dict[str, int]) -> None:
        self.set_rpm_var.set(str(status.get("set_rpm", "--")))
        self.cur_rpm_var.set(str(status.get("cur_rpm", "--")))
        self.set_temp_var.set(str(status.get("set_temp", "--")))
        self.cur_temp_var.set(str(status.get("cur_temp", "--")))

    def send_set_rpm(self) -> None:
        try:
            rpm = int(self.stir_set_rpm_var.get().strip())
            if rpm < 0 or rpm > MAX_RPM:
                raise ValueError(f"转速范围应为 0-{MAX_RPM}")
            self.stirrer_cmd_q.put(("set_rpm", rpm))
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("参数错误", str(exc))

    def send_set_temp(self) -> None:
        try:
            temp = int(self.stir_set_temp_var.get().strip())
            if temp < 0 or temp > MAX_TEMP:
                raise ValueError(f"温度范围应为 0-{MAX_TEMP}")
            self.stirrer_cmd_q.put(("set_temp", temp))
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("参数错误", str(exc))

    def send_stop_all(self) -> None:
        self.stirrer_cmd_q.put(("stop_all", 0))

    def stop(self) -> None:
        if self.stop_event:
            self.stop_event.set()
            self.log("收到停止指令")

    def on_close(self) -> None:
        self.stop()
        if self.worker_thread and self.worker_thread.is_alive():
            self.root.after(150, self.on_close)
            return
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def launch_gui() -> None:
    App().run()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LED-485-055 display controller (ASCII protocol)")
    parser.add_argument("--gui", action="store_true", help="Launch GUI control panel")
    parser.add_argument("--port", help="LED serial port, e.g. COM3")
    parser.add_argument("--address", type=int, default=1, help="LED RS485 address, default: 1")
    parser.add_argument("--baudrate", type=int, default=9600, help="LED baudrate, default: 9600")
    parser.add_argument("--digits", type=int, default=5, choices=[2, 3, 4, 5, 6], help="Display digits")
    parser.add_argument(
        "--mode",
        choices=["clock", "countdown", "stirrer"],
        default="clock",
        help="clock/date, countdown, or stirrer telemetry",
    )
    parser.add_argument("--countdown-start", type=int, default=88, help="Countdown start seconds")
    parser.add_argument("--switch-seconds", type=float, default=2.0, help="Switch interval seconds")
    parser.add_argument("--stir-port", help="Stirrer serial port, e.g. COM5")
    parser.add_argument("--stir-baudrate", type=int, default=9600, help="Stirrer baudrate")
    parser.add_argument("--poll-seconds", type=float, default=0.5, help="Stirrer poll interval")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.gui or not args.port:
        launch_gui()
        return

    config = WorkerConfig(
        port=args.port,
        address=args.address,
        baudrate=args.baudrate,
        digits=args.digits,
        mode=args.mode,
        countdown_start=args.countdown_start,
        switch_seconds=args.switch_seconds,
        stir_port=args.stir_port or "",
        stir_baudrate=args.stir_baudrate,
        poll_seconds=args.poll_seconds,
    )

    try:
        if config.mode == "clock":
            display = Led485AsciiDisplay(config.port, config.address, config.baudrate)
            try:
                run_clock(display, config.digits, config.switch_seconds, None)
            finally:
                display.close()
        elif config.mode == "countdown":
            display = Led485AsciiDisplay(config.port, config.address, config.baudrate)
            try:
                run_countdown(display, config.digits, config.countdown_start, None)
            finally:
                display.close()
        else:
            if not config.stir_port:
                parser.error("--stir-port is required when --mode stirrer")
            run_stirrer_telemetry_cli(config)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
