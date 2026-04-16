from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


BAUDRATE = 9600


class RelayClient:
    def __init__(self) -> None:
        self._serial: serial.Serial | None = None

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self, port: str) -> None:
        self.disconnect()
        self._serial = serial.Serial(
            port=port,
            baudrate=BAUDRATE,
            timeout=0.3,
            write_timeout=0.3,
        )

    def disconnect(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def set_relay(self, channel: int, turn_on: bool) -> None:
        if not 1 <= channel <= 8:
            raise ValueError("channel must be in range 1..8")
        if not self.is_connected:
            raise RuntimeError("serial port is not connected")

        state = 0x01 if turn_on else 0x00
        checksum = (0xA0 + channel + state) & 0xFF
        frame = bytes([0xA0, channel, state, checksum])
        self._serial.write(frame)

    def query_status(self) -> list[int]:
        if not self.is_connected:
            raise RuntimeError("serial port is not connected")

        self._serial.reset_input_buffer()
        self._serial.write(b"\xFF")
        raw = self._serial.read(8)
        if len(raw) != 8:
            raise RuntimeError(
                f"状态查询返回长度异常，期望 8 字节，实际 {len(raw)} 字节"
            )
        return list(raw)


class RelayGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("LCUS-8 两路继电器控制")
        self.geometry("460x300")
        self.minsize(420, 260)
        self.resizable(True, True)

        self.client = RelayClient()

        self.port_var = tk.StringVar()
        self.conn_var = tk.StringVar(value="未连接")
        self.ch1_var = tk.StringVar(value="未知")
        self.ch2_var = tk.StringVar(value="未知")

        self._build_ui()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=14)
        root.pack(fill=tk.BOTH, expand=True)

        port_row = ttk.Frame(root)
        port_row.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(port_row, text="串口").pack(side=tk.LEFT)

        self.port_combo = ttk.Combobox(
            port_row,
            textvariable=self.port_var,
            state="readonly",
            width=16,
        )
        self.port_combo.pack(side=tk.LEFT, padx=8)

        ttk.Button(port_row, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)
        ttk.Button(port_row, text="连接", command=self.connect_serial).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(port_row, text="断开", command=self.disconnect_serial).pack(side=tk.LEFT)

        info_row = ttk.Frame(root)
        info_row.pack(fill=tk.X, pady=(0, 14))

        ttk.Label(info_row, text="连接状态:").pack(side=tk.LEFT)
        ttk.Label(info_row, textvariable=self.conn_var, foreground="#0b66d6").pack(
            side=tk.LEFT, padx=6
        )

        panel = ttk.LabelFrame(root, text="两路控制", padding=12)
        panel.pack(fill=tk.BOTH, expand=True)

        self._build_channel_row(panel, channel=1, status_var=self.ch1_var)
        self._build_channel_row(panel, channel=2, status_var=self.ch2_var)

        action_row = ttk.Frame(root)
        action_row.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(action_row, text="查询状态", command=self.query_status).pack(
            side=tk.LEFT
        )
        ttk.Button(action_row, text="全部关闭(1/2路)", command=self.turn_off_both).pack(
            side=tk.LEFT, padx=8
        )

    def _build_channel_row(
        self, parent: ttk.Widget, channel: int, status_var: tk.StringVar
    ) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=8)

        ttk.Label(row, text=f"第{channel}路", width=8).pack(side=tk.LEFT)
        ttk.Button(
            row,
            text="打开",
            command=lambda c=channel: self.set_relay(c, True),
            width=8,
        ).pack(side=tk.LEFT)
        ttk.Button(
            row,
            text="关闭",
            command=lambda c=channel: self.set_relay(c, False),
            width=8,
        ).pack(side=tk.LEFT, padx=6)
        ttk.Label(row, text="状态:").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(row, textvariable=status_var, width=6).pack(side=tk.LEFT, padx=4)

    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports

        if not ports:
            self.port_var.set("")
            return

        if self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def connect_serial(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return

        try:
            self.client.connect(port)
            self.conn_var.set(f"已连接 {port} @ {BAUDRATE}")
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))

    def disconnect_serial(self) -> None:
        self.client.disconnect()
        self.conn_var.set("未连接")

    def set_relay(self, channel: int, turn_on: bool) -> None:
        try:
            self.client.set_relay(channel, turn_on)
            target = self.ch1_var if channel == 1 else self.ch2_var
            target.set("开" if turn_on else "关")
        except Exception as exc:
            messagebox.showerror("操作失败", str(exc))

    def query_status(self) -> None:
        try:
            values = self.client.query_status()
            self.ch1_var.set("开" if values[0] == 1 else "关")
            self.ch2_var.set("开" if values[1] == 1 else "关")
        except Exception as exc:
            messagebox.showerror("查询失败", str(exc))

    def turn_off_both(self) -> None:
        try:
            self.client.set_relay(1, False)
            self.client.set_relay(2, False)
            self.ch1_var.set("关")
            self.ch2_var.set("关")
        except Exception as exc:
            messagebox.showerror("操作失败", str(exc))

    def _on_close(self) -> None:
        self.client.disconnect()
        self.destroy()


def main() -> None:
    app = RelayGui()
    app.mainloop()


if __name__ == "__main__":
    main()
