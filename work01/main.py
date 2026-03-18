import serial
import time
import threading
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from serial.tools import list_ports

# ===================== 协议常量 =====================
FRAME_DOWN = 0xFE      # 上位机下传帧头
FRAME_UP   = 0xFD      # 下位机上传帧头

# 指令码
CMD_HELLO     = 0xA0   # 握手
CMD_INFO      = 0xA1   # 读取设置信息
CMD_STA       = 0xA2   # 读取状态信息
CMD_EE_READ   = 0xA3   # 读EEPROM
CMD_EE_WRITE  = 0xA4   # 写EEPROM
CMD_MOT       = 0xB1   # 搅拌控制
CMD_TEMP      = 0xB2   # 加热控制
CMD_MOD       = 0xB3   # 模式切换
CMD_SET_SAFE  = 0xB4   # 安全温度
CMD_SET_RES   = 0xB5   # 余热警告
CMD_SET_BR    = 0xB6   # 失速监测
CMD_TIMER     = 0xB7   # 定时控制
CMD_ON_OFF    = 0xB8   # 全开全关

MAX_RPM = 2000
MAX_TEMP = 300
DEFAULT_BAUDS = [9600, 19200, 38400, 57600, 115200]

# ===================== 工具函数 =====================
def calc_checksum(data):
    """计算校验和：去掉帧头的所有字节求和"""
    return sum(data[1:]) & 0xFF

def int16_to_bytes(value):
    """int转双字节：高字节在前，低字节在后"""
    high = (value >> 8) & 0xFF
    low  = value & 0xFF
    return high, low

def bytes_to_int16(high, low):
    """双字节转int"""
    return (high << 8) | low

# ===================== 主控制类 =====================
class DalongMSController:
    def __init__(self, port="COM3", baudrate=9600, timeout=0.2):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
        self.lock = threading.Lock()
        # checksum_mode:
        # - exclude_header: checksum=sum(frame[1:])，符合协议文档描述
        # - include_header: checksum=sum(frame[:])，兼容部分设备固件差异
        self.checksum_mode = "exclude_header"
        if not self.ser.is_open:
            self.ser.open()

        # 某些串口设备在打开后需要短暂稳定时间。
        time.sleep(0.15)

    def _find_response_frame(self, raw, cmd, min_len, max_len=None):
        """在原始串口返回中查找目标上行帧，并支持变长帧。"""
        if isinstance(cmd, int):
            cmd_list = [cmd]
        else:
            cmd_list = list(cmd)

        for i in range(0, len(raw) - 1):
            if raw[i] != FRAME_UP or raw[i + 1] not in cmd_list:
                continue

            available = len(raw) - i
            if available < min_len:
                continue

            search_max = available if max_len is None else min(available, max_len)

            # 优先尝试更长帧，避免把长帧误截成短帧。
            for frame_len in range(search_max, min_len - 1, -1):
                candidate = raw[i : i + frame_len]
                if self._validate_checksum(candidate):
                    return candidate
        return b""

    def _validate_checksum(self, frame):
        if len(frame) < 2:
            return False
        expected_exclude = sum(frame[1:-1]) & 0xFF
        expected_include = sum(frame[:-1]) & 0xFF
        # 接收侧放宽：兼容两种固件实现
        return frame[-1] in (expected_exclude, expected_include)

    def _calc_checksum_for_mode(self, frame_without_checksum, mode):
        if mode == "include_header":
            return sum(frame_without_checksum) & 0xFF
        return sum(frame_without_checksum[1:]) & 0xFF

    def _find_frame_no_checksum(self, raw, cmd, min_len):
        """仅按帧头+指令定位，不校验checksum，用于诊断。"""
        for i in range(0, len(raw) - 1):
            if raw[i] == FRAME_UP and raw[i + 1] == cmd and (len(raw) - i) >= min_len:
                return raw[i : i + min_len]
        return b""

    def _bytes_hex(self, data):
        return " ".join(f"{b:02X}" for b in data)

    def _read_raw_with_deadline(self, total_timeout=0.4):
        """在限定时间内读取串口数据并返回原始字节流。"""
        deadline = time.time() + total_timeout
        chunks = bytearray()
        while time.time() < deadline:
            waiting = self.ser.in_waiting
            if waiting > 0:
                chunks.extend(self.ser.read(waiting))
            else:
                chunks.extend(self.ser.read(1))

            # 如果已经读到数据，再给设备一点点时间把后续字节补齐。
            if chunks:
                time.sleep(0.01)
        return bytes(chunks)

    def send_cmd(self, cmd, param1=0, param2=0, param3=0, expected_len=6, total_timeout=0.4, expected_cmd=None, max_len=None, checksum_mode=None):
        """发送6字节下传指令：帧头+指令+3参数+校验和"""
        frame = [FRAME_DOWN, cmd, param1, param2, param3]
        mode = checksum_mode or self.checksum_mode
        checksum = self._calc_checksum_for_mode(frame, mode)
        frame.append(checksum)

        with self.lock:
            # 发送前清空接收缓冲，避免读到上一次轮询残留帧。
            self.ser.reset_input_buffer()
            self.ser.write(bytes(frame))
            self.ser.flush()
            raw = self._read_raw_with_deadline(total_timeout=total_timeout)

        response_cmd = cmd if expected_cmd is None else expected_cmd
        return self._find_response_frame(raw, response_cmd, expected_len, max_len=max_len)

    def send_cmd_debug(self, cmd, param1=0, param2=0, param3=0, expected_len=6, total_timeout=0.4, expected_cmd=None, max_len=None, checksum_mode=None):
        """发送指令并返回诊断信息：发送帧、原始回包、解析帧。"""
        frame = [FRAME_DOWN, cmd, param1, param2, param3]
        mode = checksum_mode or self.checksum_mode
        checksum = self._calc_checksum_for_mode(frame, mode)
        frame.append(checksum)

        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(bytes(frame))
            self.ser.flush()
            raw = self._read_raw_with_deadline(total_timeout=total_timeout)

        response_cmd = cmd if expected_cmd is None else expected_cmd
        parsed = self._find_response_frame(raw, response_cmd, expected_len, max_len=max_len)
        loose_cmd = response_cmd[0] if isinstance(response_cmd, (list, tuple)) else response_cmd
        loose = self._find_frame_no_checksum(raw, loose_cmd, expected_len)
        return {
            "tx": bytes(frame),
            "raw": raw,
            "parsed": parsed,
            "loose": loose,
            "checksum_mode": mode,
        }

    def hello(self):
        """握手"""
        resp = self.send_cmd(CMD_HELLO, 0, 0, 0, expected_len=6, total_timeout=0.8)
        if len(resp) >= 6:
            return resp[2] == 0  # 返回码=0 表示成功
        return False

    def hello_diagnose(self, retries=3, delay=0.12):
        """握手诊断：重试并返回可读的失败原因。"""
        attempts = []
        # 先使用当前模式，再自动尝试另一种校验模式。
        mode_candidates = [self.checksum_mode, "include_header" if self.checksum_mode == "exclude_header" else "exclude_header"]

        attempt_index = 0
        for mode in mode_candidates:
            for retry_idx in range(retries):
                attempt_index += 1
                info = self.send_cmd_debug(
                    CMD_HELLO,
                    0,
                    0,
                    0,
                    expected_len=6,
                    total_timeout=0.8,
                    checksum_mode=mode,
                )
                parsed = info["parsed"]
                loose = info["loose"]

                item = {
                    "attempt": attempt_index,
                    "tx_hex": self._bytes_hex(info["tx"]),
                    "raw_hex": self._bytes_hex(info["raw"]),
                    "frame_hex": self._bytes_hex(parsed) if parsed else "",
                    "checksum_mode": info["checksum_mode"],
                }

                if parsed:
                    code = parsed[2]
                    item["result"] = "ok" if code == 0 else "device_nack"
                    item["code"] = code
                    attempts.append(item)
                    # 记住可用校验方式，后续普通指令复用。
                    self.checksum_mode = info["checksum_mode"]
                    return {
                        "ok": code == 0,
                        "reason": "success" if code == 0 else f"device return code={code}",
                        "attempts": attempts,
                    }

                if info["raw"] and loose and not parsed:
                    item["result"] = "checksum_error"
                elif info["raw"]:
                    item["result"] = "unexpected_frame"
                else:
                    item["result"] = "no_response"

                attempts.append(item)
                if retry_idx < retries - 1:
                    time.sleep(delay)

        last = attempts[-1]["result"] if attempts else "unknown"
        if last == "checksum_error":
            reason = "收到握手帧但校验失败"
        elif last == "unexpected_frame":
            reason = "串口有回包但不是握手帧"
        elif last == "no_response":
            reason = "设备无回包"
        else:
            reason = "握手失败"

        return {
            "ok": False,
            "reason": reason,
            "attempts": attempts,
        }

    def set_stir(self, rpm):
        """设置搅拌转速 rpm=0则停止"""
        h, l = int16_to_bytes(rpm)
        resp = self.send_cmd(CMD_MOT, h, l, 0, expected_len=6)
        if len(resp) >= 6:
            return resp[2] == 0
        return False

    def set_heat(self, temp):
        """设置加热温度 temp=0则停止加热"""
        h, l = int16_to_bytes(temp)
        resp = self.send_cmd(CMD_TEMP, h, l, 0, expected_len=6)
        if len(resp) >= 6:
            return resp[2] == 0
        return False

    def get_status(self):
        """读取实时状态：转速、温度、定时等"""
        # PDF里状态上行可能是11字节(8参数)或15字节(12参数)，且文档中CMD字段存在A1/A2混写，做兼容。
        resp = self.send_cmd(
            CMD_STA,
            0,
            0,
            0,
            expected_len=11,
            expected_cmd=[CMD_STA, CMD_INFO],
            max_len=15,
        )
        if len(resp) >= 11:
            # 协议参数1从索引2开始。
            set_rpm = bytes_to_int16(resp[2], resp[3])
            cur_rpm = bytes_to_int16(resp[4], resp[5])
            set_temp = bytes_to_int16(resp[6], resp[7])
            cur_temp = bytes_to_int16(resp[8], resp[9])

            data = {
                "设定转速": set_rpm,
                "实际转速": cur_rpm,
                "设定温度": set_temp,
                "实际温度": cur_temp,
            }

            if len(resp) >= 15:
                data["设定定时"] = bytes_to_int16(resp[10], resp[11])
                data["剩余时间"] = bytes_to_int16(resp[12], resp[13])

            return data
        return None

    def close(self):
        self.ser.close()

# ===================== GUI界面 =====================
class DalongMSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dalong MS 智能控制台")
        self.root.geometry("640x560")
        self.root.minsize(640, 560)

        self.controller = None
        self.is_polling = False

        self.port_var = tk.StringVar(value="COM3")
        self.baud_var = tk.StringVar(value="9600")
        self.rpm_var = tk.StringVar(value="500")
        self.temp_var = tk.StringVar(value="50")

        self.status_text_var = tk.StringVar(value="未连接")
        self.set_rpm_var = tk.StringVar(value="--")
        self.cur_rpm_var = tk.StringVar(value="--")
        self.set_temp_var = tk.StringVar(value="--")
        self.cur_temp_var = tk.StringVar(value="--")

        self.status_color_var = tk.StringVar(value="#9b2c2c")

        self.btn_set_stir = None
        self.btn_set_heat = None
        self.btn_stop = None
        self.btn_handshake = None
        self.btn_refresh_once = None
        self.btn_auto_detect = None
        self.log_text = None

        self._build_ui()
        self.refresh_ports()
        self._set_controls_enabled(False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        style = ttk.Style()
        try:
            style.theme_use("vista")
        except tk.TclError:
            pass

        conn_frame = ttk.LabelFrame(self.root, text="连接设置")
        conn_frame.pack(fill="x", padx=12, pady=10)

        ttk.Label(conn_frame, text="串口:").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=6, pady=6, sticky="w")

        ttk.Button(conn_frame, text="刷新串口", command=self.refresh_ports).grid(row=0, column=2, padx=6, pady=6)
        self.btn_auto_detect = ttk.Button(conn_frame, text="自动探测", command=self.auto_detect)
        self.btn_auto_detect.grid(row=0, column=3, padx=6, pady=6)

        ttk.Label(conn_frame, text="波特率:").grid(row=1, column=0, padx=6, pady=6, sticky="w")
        self.baud_combo = ttk.Combobox(
            conn_frame,
            textvariable=self.baud_var,
            values=["9600", "19200", "38400", "57600", "115200"],
            width=12,
            state="readonly",
        )
        self.baud_combo.grid(row=1, column=1, padx=6, pady=6, sticky="w")

        ttk.Button(conn_frame, text="连接", command=self.connect).grid(row=1, column=2, padx=6, pady=6)
        ttk.Button(conn_frame, text="断开", command=self.disconnect).grid(row=1, column=3, padx=6, pady=6)

        op_frame = ttk.LabelFrame(self.root, text="控制")
        op_frame.pack(fill="x", padx=12, pady=8)

        ttk.Label(op_frame, text="转速 RPM:").grid(row=0, column=0, padx=6, pady=8, sticky="w")
        ttk.Spinbox(op_frame, textvariable=self.rpm_var, from_=0, to=MAX_RPM, increment=50, width=10).grid(row=0, column=1, padx=6, pady=8)
        self.btn_set_stir = ttk.Button(op_frame, text="设置搅拌", command=self.apply_stir)
        self.btn_set_stir.grid(row=0, column=2, padx=6, pady=8)

        ttk.Label(op_frame, text="温度 ℃:").grid(row=1, column=0, padx=6, pady=8, sticky="w")
        ttk.Spinbox(op_frame, textvariable=self.temp_var, from_=0, to=MAX_TEMP, increment=5, width=10).grid(row=1, column=1, padx=6, pady=8)
        self.btn_set_heat = ttk.Button(op_frame, text="设置加热", command=self.apply_heat)
        self.btn_set_heat.grid(row=1, column=2, padx=6, pady=8)

        self.btn_stop = ttk.Button(op_frame, text="一键停止", command=self.stop_all)
        self.btn_stop.grid(row=0, column=3, rowspan=2, padx=10, pady=8, sticky="ns")
        self.btn_handshake = ttk.Button(op_frame, text="握手测试", command=self.handshake)
        self.btn_handshake.grid(row=0, column=4, rowspan=2, padx=6, pady=8, sticky="ns")

        sta_frame = ttk.LabelFrame(self.root, text="实时状态")
        sta_frame.pack(fill="x", padx=12, pady=8)

        ttk.Label(sta_frame, text="设定转速:").grid(row=0, column=0, padx=8, pady=6, sticky="w")
        ttk.Label(sta_frame, textvariable=self.set_rpm_var).grid(row=0, column=1, padx=8, pady=6, sticky="w")

        ttk.Label(sta_frame, text="实际转速:").grid(row=0, column=2, padx=8, pady=6, sticky="w")
        ttk.Label(sta_frame, textvariable=self.cur_rpm_var).grid(row=0, column=3, padx=8, pady=6, sticky="w")

        ttk.Label(sta_frame, text="设定温度:").grid(row=1, column=0, padx=8, pady=6, sticky="w")
        ttk.Label(sta_frame, textvariable=self.set_temp_var).grid(row=1, column=1, padx=8, pady=6, sticky="w")

        ttk.Label(sta_frame, text="实际温度:").grid(row=1, column=2, padx=8, pady=6, sticky="w")
        ttk.Label(sta_frame, textvariable=self.cur_temp_var).grid(row=1, column=3, padx=8, pady=6, sticky="w")
        self.btn_refresh_once = ttk.Button(sta_frame, text="立即刷新", command=self.refresh_status_once)
        self.btn_refresh_once.grid(row=0, column=4, rowspan=2, padx=8, pady=6, sticky="ns")

        tip_frame = ttk.Frame(self.root)
        tip_frame.pack(fill="x", padx=12, pady=(2, 10))
        ttk.Label(tip_frame, text="连接状态:").pack(side="left")
        self.status_label = tk.Label(tip_frame, textvariable=self.status_text_var, fg="#9b2c2c")
        self.status_label.pack(side="left", padx=6)

        log_frame = ttk.LabelFrame(self.root, text="运行日志")
        log_frame.pack(fill="both", expand=True, padx=12, pady=(0, 10))
        self.log_text = ScrolledText(log_frame, height=10, state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

    def _log(self, text):
        if self.log_text is None:
            return
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] {text}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _set_status(self, text, color="#0b6a0b"):
        self.status_text_var.set(text)
        self.status_label.configure(fg=color)

    def _set_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        for btn in [self.btn_set_stir, self.btn_set_heat, self.btn_stop, self.btn_handshake, self.btn_refresh_once]:
            if btn is not None:
                btn.configure(state=state)

    def auto_detect(self):
        if self.controller is not None:
            messagebox.showinfo("提示", "请先断开当前连接后再自动探测。")
            return

        ports = [p.device for p in list_ports.comports()]
        if not ports:
            messagebox.showwarning("无串口", "未检测到可用串口。")
            return

        self._log(f"开始自动探测，端口={ports}，波特率={DEFAULT_BAUDS}")
        self.root.config(cursor="watch")
        self.root.update_idletasks()

        try:
            for port in ports:
                for baud in DEFAULT_BAUDS:
                    self._log(f"探测 {port} @ {baud}")
                    test_ctrl = None
                    try:
                        test_ctrl = DalongMSController(port=port, baudrate=baud)
                        diag = test_ctrl.hello_diagnose(retries=1, delay=0.05)
                        if diag["ok"]:
                            self.port_var.set(port)
                            self.baud_var.set(str(baud))
                            self._log(f"探测成功: {port} @ {baud}")
                            messagebox.showinfo("探测成功", f"找到可通信参数:\n串口: {port}\n波特率: {baud}")
                            return

                        if diag["attempts"]:
                            last = diag["attempts"][-1]
                            self._log(
                                f"未通过: {port}@{baud} reason={diag['reason']} "
                                f"mode={last.get('checksum_mode', 'unknown')} raw={last.get('raw_hex', '')}"
                            )
                        else:
                            self._log(f"未通过: {port}@{baud} reason={diag['reason']}")
                    except Exception as e:
                        self._log(f"探测异常: {port}@{baud} err={e}")
                    finally:
                        if test_ctrl is not None:
                            try:
                                test_ctrl.close()
                            except Exception:
                                pass

            messagebox.showwarning("探测失败", "未找到可握手的串口参数。\n请检查线材、接线、设备通讯模式。")
        finally:
            self.root.config(cursor="")

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        if not ports:
            ports = ["COM3"]
        self.port_combo["values"] = ports
        if self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self._log("串口列表已刷新")

    def _require_connection(self):
        if self.controller is None:
            messagebox.showwarning("未连接", "请先连接串口。")
            return False
        return True

    def connect(self):
        if self.controller is not None:
            messagebox.showinfo("提示", "当前已连接。")
            return

        try:
            self.controller = DalongMSController(
                port=self.port_var.get(),
                baudrate=int(self.baud_var.get()),
            )
            self._set_status(f"已连接 {self.port_var.get()}", "#0b6a0b")
            self._set_controls_enabled(True)
            self._log(f"串口连接成功: {self.port_var.get()} @ {self.baud_var.get()}")

            # 连接后做一次握手诊断，失败时给出具体原因。
            hello_diag = self.controller.hello_diagnose(retries=2)
            if not hello_diag["ok"]:
                self._log(f"握手失败: {hello_diag['reason']}")
                for item in hello_diag["attempts"]:
                    self._log(
                        f"握手尝试#{item['attempt']} mode={item.get('checksum_mode', 'unknown')} "
                        f"result={item['result']} tx={item['tx_hex']} raw={item['raw_hex']}"
                    )
                messagebox.showwarning(
                    "连接成功但握手失败",
                    f"串口已连接，但设备未正确响应握手。\n原因: {hello_diag['reason']}",
                )
            else:
                self._log("握手成功")

            self.start_polling()
        except Exception as e:
            self.controller = None
            self._set_controls_enabled(False)
            self._set_status("未连接", "#9b2c2c")
            self._log(f"连接失败: {e}")
            messagebox.showerror("连接失败", str(e))

    def disconnect(self):
        self.stop_polling()
        if self.controller is not None:
            try:
                self.controller.close()
            except Exception:
                pass
            self.controller = None
        self._set_controls_enabled(False)
        self._set_status("未连接", "#9b2c2c")
        self._log("串口已断开")

    def handshake(self):
        if not self._require_connection():
            return
        try:
            diag = self.controller.hello_diagnose(retries=3)
            if diag["ok"]:
                self._log("手动握手成功")
                messagebox.showinfo("握手", "握手成功")
            else:
                self._log(f"手动握手失败: {diag['reason']}")
                for item in diag["attempts"]:
                    self._log(
                        f"握手尝试#{item['attempt']} mode={item.get('checksum_mode', 'unknown')} "
                        f"result={item['result']} tx={item['tx_hex']} raw={item['raw_hex']}"
                    )
                messagebox.showwarning("握手", f"握手失败\n原因: {diag['reason']}")
        except Exception as e:
            self._log(f"握手异常: {e}")
            messagebox.showerror("握手异常", str(e))

    def apply_stir(self):
        if not self._require_connection():
            return
        try:
            rpm = int(self.rpm_var.get())
            if rpm < 0 or rpm > MAX_RPM:
                raise ValueError(f"转速范围应为 0~{MAX_RPM} RPM")
            if not self.controller.set_stir(rpm):
                self._log(f"搅拌设置失败: {rpm} RPM")
                messagebox.showwarning("设置失败", "搅拌设置未确认成功")
            else:
                self._log(f"已设置搅拌: {rpm} RPM")
        except Exception as e:
            messagebox.showerror("参数错误", str(e))

    def apply_heat(self):
        if not self._require_connection():
            return
        try:
            temp = int(self.temp_var.get())
            if temp < 0 or temp > MAX_TEMP:
                raise ValueError(f"温度范围应为 0~{MAX_TEMP} ℃")
            if not self.controller.set_heat(temp):
                self._log(f"加热设置失败: {temp} ℃")
                messagebox.showwarning("设置失败", "加热设置未确认成功")
            else:
                self._log(f"已设置加热: {temp} ℃")
        except Exception as e:
            messagebox.showerror("参数错误", str(e))

    def stop_all(self):
        if not self._require_connection():
            return
        try:
            ok_stir = self.controller.set_stir(0)
            ok_heat = self.controller.set_heat(0)
            if ok_stir and ok_heat:
                self._log("已执行一键停止")
            else:
                self._log("一键停止已发送，但设备未完全确认")
        except Exception as e:
            self._log(f"停止失败: {e}")
            messagebox.showerror("停止失败", str(e))

    def refresh_status_once(self):
        if not self._require_connection():
            return
        self.poll_status()

    def poll_status(self):
        if self.controller is None:
            return

        try:
            status = self.controller.get_status()
            if status:
                self.set_rpm_var.set(str(status["设定转速"]))
                self.cur_rpm_var.set(str(status["实际转速"]))
                self.set_temp_var.set(str(status["设定温度"]))
                self.cur_temp_var.set(str(status["实际温度"]))
        except Exception:
            # 刷新过程中偶发超时不弹窗，避免打断操作。
            pass

        if self.is_polling:
            self.root.after(1000, self.poll_status)

    def start_polling(self):
        if not self.is_polling:
            self.is_polling = True
            self.poll_status()

    def stop_polling(self):
        self.is_polling = False

    def on_close(self):
        self.disconnect()
        self.root.destroy()


# ===================== 命令行测试示例 =====================
def run_cli_demo():
    ms = None
    try:
        # 1. 初始化（请改串口号）
        ms = DalongMSController(port="COM3", baudrate=9600)
        print("串口打开成功")

        # 2. 握手
        if ms.hello():
            print("✅ 握手成功")
        else:
            print("❌ 握手失败")

        # 3. 启动搅拌 500转
        ms.set_stir(500)
        print("✅ 已设置搅拌 500 RPM")

        time.sleep(2)

        # 4. 启动加热 50度
        ms.set_heat(50)
        print("✅ 已设置加热 50 ℃")

        time.sleep(2)

        # 5. 读取状态
        status = ms.get_status()
        if status:
            print("📊 当前状态：", status)

        # 6. 停止
        ms.set_stir(0)
        ms.set_heat(0)
        print("✅ 已停止搅拌与加热")

    except Exception as e:
        print("异常：", e)
    finally:
        if ms:
            ms.close()


if __name__ == "__main__":
    root = tk.Tk()
    app = DalongMSApp(root)
    root.mainloop()