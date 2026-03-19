from __future__ import annotations

import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext, ttk

import serial
from serial.tools import list_ports


def crc16_modbus(data: bytes) -> int:
	crc = 0xFFFF
	for byte in data:
		crc ^= byte
		for _ in range(8):
			if crc & 1:
				crc = (crc >> 1) ^ 0xA001
			else:
				crc >>= 1
	return crc & 0xFFFF


def with_crc(frame_wo_crc: bytes) -> bytes:
	crc = crc16_modbus(frame_wo_crc)
	return frame_wo_crc + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


@dataclass
class SensorConfig:
	slave_id: int = 1
	baudrate: int = 9600
	timeout: float = 0.35
	temp_reg: int = 0
	pt100_reg: int = 1
	scale: float = 10.0
	signed: bool = True


class PTA9B01Client:
	def __init__(self) -> None:
		self.ser: serial.Serial | None = None
		self.lock = threading.Lock()

	@property
	def is_connected(self) -> bool:
		return self.ser is not None and self.ser.is_open

	def connect(self, port: str, baudrate: int, timeout: float) -> None:
		self.disconnect()
		self.ser = serial.Serial(
			port=port,
			baudrate=baudrate,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			timeout=timeout,
			write_timeout=timeout,
		)

	def disconnect(self) -> None:
		if self.ser and self.ser.is_open:
			self.ser.close()
		self.ser = None

	def _xfer(self, req: bytes, expected_len: int) -> bytes:
		if not self.is_connected:
			raise RuntimeError("串口未连接")

		assert self.ser is not None
		with self.lock:
			self.ser.reset_input_buffer()
			self.ser.write(req)
			self.ser.flush()
			resp = self.ser.read(expected_len)

		if len(resp) != expected_len:
			raise RuntimeError(f"响应长度异常: 期望 {expected_len}，实际 {len(resp)}")
		if crc16_modbus(resp[:-2]) != int.from_bytes(resp[-2:], "little"):
			raise RuntimeError("CRC 校验失败")
		return resp

	def read_holding(self, slave_id: int, reg_addr: int, count: int = 1) -> list[int]:
		if not (1 <= slave_id <= 247):
			raise ValueError("从站地址范围应为 1..247")
		if not (0 <= reg_addr <= 0xFFFF):
			raise ValueError("寄存器地址范围应为 0..65535")
		if not (1 <= count <= 125):
			raise ValueError("读取寄存器数量范围应为 1..125")

		frame = bytes(
			[
				slave_id,
				0x03,
				(reg_addr >> 8) & 0xFF,
				reg_addr & 0xFF,
				(count >> 8) & 0xFF,
				count & 0xFF,
			]
		)
		req = with_crc(frame)
		expected_len = 5 + count * 2
		resp = self._xfer(req, expected_len)

		if resp[0] != slave_id or resp[1] != 0x03:
			raise RuntimeError(f"功能码异常: {resp.hex(' ').upper()}")
		if resp[2] != count * 2:
			raise RuntimeError(f"字节数异常: {resp[2]}")

		values: list[int] = []
		for i in range(count):
			idx = 3 + i * 2
			values.append((resp[idx] << 8) | resp[idx + 1])
		return values

	def write_single(self, slave_id: int, reg_addr: int, value: int) -> None:
		if not (1 <= slave_id <= 247):
			raise ValueError("从站地址范围应为 1..247")
		if not (0 <= reg_addr <= 0xFFFF):
			raise ValueError("寄存器地址范围应为 0..65535")
		if not (0 <= value <= 0xFFFF):
			raise ValueError("寄存器值范围应为 0..65535")

		frame = bytes(
			[
				slave_id,
				0x06,
				(reg_addr >> 8) & 0xFF,
				reg_addr & 0xFF,
				(value >> 8) & 0xFF,
				value & 0xFF,
			]
		)
		req = with_crc(frame)
		resp = self._xfer(req, expected_len=8)
		if resp != req:
			raise RuntimeError("写寄存器确认帧不匹配")


class App(tk.Tk):
	def __init__(self) -> None:
		super().__init__()
		self.title("PTA9B01 PT100 传感器控制台")
		self.geometry("900x600")
		self.minsize(780, 500)

		self.client = PTA9B01Client()
		self.stop_event = threading.Event()
		self.poll_thread: threading.Thread | None = None

		self.port_var = tk.StringVar()
		self.baud_var = tk.StringVar(value="9600")
		self.timeout_var = tk.StringVar(value="0.35")
		self.slave_var = tk.StringVar(value="1")
		self.temp_reg_var = tk.StringVar(value="0")
		self.pt100_reg_var = tk.StringVar(value="1")
		self.scale_var = tk.StringVar(value="10")
		self.signed_var = tk.BooleanVar(value=True)

		self.poll_enable_var = tk.BooleanVar(value=False)
		self.poll_interval_var = tk.StringVar(value="1.0")

		self.temp_value_var = tk.StringVar(value="--")
		self.raw_temp_var = tk.StringVar(value="--")
		self.pt100_value_var = tk.StringVar(value="--")
		self.conn_var = tk.StringVar(value="未连接")

		self.new_id_var = tk.StringVar(value="1")
		self.id_reg_var = tk.StringVar(value="2")

		self._build_ui()
		self.refresh_ports()
		self.protocol("WM_DELETE_WINDOW", self.on_close)

	def _build_ui(self) -> None:
		self.columnconfigure(0, weight=1)
		self.rowconfigure(1, weight=1)

		top = ttk.Frame(self, padding=10)
		top.grid(row=0, column=0, sticky="ew")
		for c in range(9):
			top.columnconfigure(c, weight=0)
		top.columnconfigure(8, weight=1)

		ttk.Label(top, text="串口").grid(row=0, column=0, sticky="w")
		self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=10, state="readonly")
		self.port_combo.grid(row=0, column=1, padx=6, sticky="w")
		ttk.Button(top, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=2)

		ttk.Label(top, text="波特率").grid(row=0, column=3, padx=(10, 0), sticky="w")
		ttk.Entry(top, textvariable=self.baud_var, width=8).grid(row=0, column=4, padx=4, sticky="w")

		ttk.Label(top, text="超时(s)").grid(row=0, column=5, padx=(10, 0), sticky="w")
		ttk.Entry(top, textvariable=self.timeout_var, width=6).grid(row=0, column=6, padx=4, sticky="w")

		ttk.Button(top, text="连接", command=self.connect_serial).grid(row=0, column=7, padx=4)
		ttk.Button(top, text="断开", command=self.disconnect_serial).grid(row=0, column=8, padx=4, sticky="w")

		ttk.Label(top, textvariable=self.conn_var, foreground="#0b66d6").grid(
			row=1, column=0, columnspan=9, sticky="w", pady=(6, 0)
		)

		body = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
		body.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))

		left = ttk.Frame(body, padding=10)
		right = ttk.Frame(body, padding=10)
		body.add(left, weight=3)
		body.add(right, weight=2)

		self._build_left(left)
		self._build_right(right)

	def _build_left(self, parent: ttk.Frame) -> None:
		parent.columnconfigure(1, weight=1)
		row = 0

		ttk.Label(parent, text="从站地址").grid(row=row, column=0, sticky="w")
		ttk.Entry(parent, textvariable=self.slave_var, width=10).grid(row=row, column=1, sticky="w")
		row += 1

		ttk.Label(parent, text="温度寄存器").grid(row=row, column=0, sticky="w", pady=(8, 0))
		ttk.Entry(parent, textvariable=self.temp_reg_var, width=10).grid(row=row, column=1, sticky="w", pady=(8, 0))
		row += 1

		ttk.Label(parent, text="PT100值寄存器").grid(row=row, column=0, sticky="w", pady=(8, 0))
		ttk.Entry(parent, textvariable=self.pt100_reg_var, width=10).grid(row=row, column=1, sticky="w", pady=(8, 0))
		row += 1

		ttk.Label(parent, text="温度缩放因子").grid(row=row, column=0, sticky="w", pady=(8, 0))
		ttk.Entry(parent, textvariable=self.scale_var, width=10).grid(row=row, column=1, sticky="w", pady=(8, 0))
		row += 1

		ttk.Checkbutton(parent, text="温度按有符号16位解析", variable=self.signed_var).grid(
			row=row, column=0, columnspan=2, sticky="w", pady=(8, 0)
		)
		row += 1

		btn_row = ttk.Frame(parent)
		btn_row.grid(row=row, column=0, columnspan=2, sticky="w", pady=(12, 0))
		ttk.Button(btn_row, text="读取一次", command=self.read_once).pack(side=tk.LEFT)
		ttk.Checkbutton(btn_row, text="轮询", variable=self.poll_enable_var, command=self.toggle_poll).pack(side=tk.LEFT, padx=(8, 0))
		ttk.Label(btn_row, text="间隔(s)").pack(side=tk.LEFT, padx=(8, 0))
		ttk.Entry(btn_row, textvariable=self.poll_interval_var, width=6).pack(side=tk.LEFT, padx=(2, 0))
		row += 1

		card = ttk.LabelFrame(parent, text="实时读数", padding=10)
		card.grid(row=row, column=0, columnspan=2, sticky="nsew", pady=(14, 0))
		card.columnconfigure(1, weight=1)

		ttk.Label(card, text="温度(°C)").grid(row=0, column=0, sticky="w")
		ttk.Label(card, textvariable=self.temp_value_var, foreground="#156c2f", font=("Segoe UI", 15, "bold")).grid(
			row=0, column=1, sticky="w"
		)
		ttk.Label(card, text="原始温度值").grid(row=1, column=0, sticky="w", pady=(6, 0))
		ttk.Label(card, textvariable=self.raw_temp_var).grid(row=1, column=1, sticky="w", pady=(6, 0))
		ttk.Label(card, text="PT100原始值").grid(row=2, column=0, sticky="w", pady=(6, 0))
		ttk.Label(card, textvariable=self.pt100_value_var).grid(row=2, column=1, sticky="w", pady=(6, 0))

	def _build_right(self, parent: ttk.Frame) -> None:
		parent.columnconfigure(1, weight=1)

		id_box = ttk.LabelFrame(parent, text="设置从站地址 (功能码06)", padding=10)
		id_box.grid(row=0, column=0, sticky="ew")

		ttk.Label(id_box, text="地址寄存器").grid(row=0, column=0, sticky="w")
		ttk.Entry(id_box, textvariable=self.id_reg_var, width=10).grid(row=0, column=1, sticky="w")
		ttk.Label(id_box, text="新地址(1-247)").grid(row=1, column=0, sticky="w", pady=(8, 0))
		ttk.Entry(id_box, textvariable=self.new_id_var, width=10).grid(row=1, column=1, sticky="w", pady=(8, 0))
		ttk.Button(id_box, text="写入地址", command=self.write_new_slave_id).grid(
			row=2, column=0, columnspan=2, sticky="w", pady=(10, 0)
		)

		tip = (
			"说明:\n"
			"1) 本程序按手册使用 Modbus RTU 03/06。\n"
			"2) 当前手册附件未包含完整寄存器表，\n"
			"   温度/电阻/地址寄存器均可在界面中改。\n"
			"3) 默认值: 温度=0, PT100=1, 地址寄存器=2。"
		)
		ttk.Label(parent, text=tip, justify=tk.LEFT).grid(row=1, column=0, sticky="w", pady=(12, 8))

		log_box = ttk.LabelFrame(parent, text="通信日志", padding=8)
		log_box.grid(row=2, column=0, sticky="nsew")
		parent.rowconfigure(2, weight=1)

		self.log_widget = scrolledtext.ScrolledText(log_box, width=45, height=20)
		self.log_widget.pack(fill=tk.BOTH, expand=True)

	def refresh_ports(self) -> None:
		ports = [p.device for p in list_ports.comports()]
		self.port_combo["values"] = ports
		if ports and self.port_var.get() not in ports:
			self.port_var.set(ports[0])
		if not ports:
			self.port_var.set("")

	def _parse_config(self) -> SensorConfig:
		return SensorConfig(
			slave_id=int(self.slave_var.get().strip()),
			baudrate=int(self.baud_var.get().strip()),
			timeout=float(self.timeout_var.get().strip()),
			temp_reg=int(self.temp_reg_var.get().strip()),
			pt100_reg=int(self.pt100_reg_var.get().strip()),
			scale=float(self.scale_var.get().strip()),
			signed=self.signed_var.get(),
		)

	def connect_serial(self) -> None:
		try:
			port = self.port_var.get().strip()
			if not port:
				raise ValueError("请先选择串口")
			cfg = self._parse_config()
			self.client.connect(port, cfg.baudrate, cfg.timeout)
			self.conn_var.set(f"已连接 {port} @ {cfg.baudrate} N81")
			self.log(f"连接成功: {port}, baud={cfg.baudrate}, timeout={cfg.timeout}")
		except Exception as exc:
			messagebox.showerror("连接失败", str(exc))

	def disconnect_serial(self) -> None:
		self.stop_poll_thread()
		self.client.disconnect()
		self.conn_var.set("未连接")
		self.log("串口已断开")

	def _decode_temp(self, raw: int, cfg: SensorConfig) -> float:
		value = raw
		if cfg.signed and raw >= 0x8000:
			value = raw - 0x10000
		return value / cfg.scale

	def read_once(self) -> None:
		try:
			cfg = self._parse_config()
			temp_raw = self.client.read_holding(cfg.slave_id, cfg.temp_reg, 1)[0]
			pt100_raw = self.client.read_holding(cfg.slave_id, cfg.pt100_reg, 1)[0]
			temp_c = self._decode_temp(temp_raw, cfg)

			self.temp_value_var.set(f"{temp_c:.2f}")
			self.raw_temp_var.set(str(temp_raw))
			self.pt100_value_var.set(str(pt100_raw))
			self.log(
				f"读数成功: slave={cfg.slave_id}, temp_reg={cfg.temp_reg}->{temp_raw}, "
				f"pt100_reg={cfg.pt100_reg}->{pt100_raw}, temp={temp_c:.2f}C"
			)
		except Exception as exc:
			self.log(f"读取失败: {exc}")

	def toggle_poll(self) -> None:
		if self.poll_enable_var.get():
			try:
				interval = float(self.poll_interval_var.get().strip())
				if interval <= 0:
					raise ValueError("轮询间隔必须大于0")
				self.stop_event.clear()
				self.poll_thread = threading.Thread(target=self.poll_worker, daemon=True)
				self.poll_thread.start()
				self.log(f"开始轮询, interval={interval}s")
			except Exception as exc:
				self.poll_enable_var.set(False)
				messagebox.showerror("轮询启动失败", str(exc))
		else:
			self.stop_poll_thread()

	def poll_worker(self) -> None:
		while not self.stop_event.is_set():
			self.after(0, self.read_once)
			try:
				interval = float(self.poll_interval_var.get().strip())
			except Exception:
				interval = 1.0
			self.stop_event.wait(interval)

	def stop_poll_thread(self) -> None:
		self.stop_event.set()
		if self.poll_thread and self.poll_thread.is_alive():
			self.poll_thread.join(timeout=1.0)
		self.poll_thread = None
		if self.poll_enable_var.get():
			self.poll_enable_var.set(False)
		self.log("轮询停止")

	def write_new_slave_id(self) -> None:
		try:
			cfg = self._parse_config()
			reg = int(self.id_reg_var.get().strip())
			new_id = int(self.new_id_var.get().strip())
			self.client.write_single(cfg.slave_id, reg, new_id)
			self.slave_var.set(str(new_id))
			self.log(f"从站地址写入成功: old={cfg.slave_id} -> new={new_id}, reg={reg}")
			messagebox.showinfo("成功", "地址写入完成，建议断电重启设备后重连")
		except Exception as exc:
			messagebox.showerror("写入失败", str(exc))
			self.log(f"地址写入失败: {exc}")

	def log(self, text: str) -> None:
		ts = time.strftime("%H:%M:%S")
		self.log_widget.insert(tk.END, f"[{ts}] {text}\n")
		self.log_widget.see(tk.END)

	def on_close(self) -> None:
		self.stop_poll_thread()
		self.client.disconnect()
		self.destroy()


def main() -> None:
	app = App()
	app.mainloop()


if __name__ == "__main__":
	main()
