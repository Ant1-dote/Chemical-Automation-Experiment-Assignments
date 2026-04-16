from dataclasses import dataclass
import sys

from pymodbus.client import ModbusSerialClient
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


MODE_LABEL_TO_VALUE = {
    "普通模式": 0,
    "点动模式": 1,
    "延时开启": 2,
    "PWM模式": 3,
}

MODE_VALUE_TO_LABEL = {value: key for key, value in MODE_LABEL_TO_VALUE.items()}

BAUD_INDEX_TO_VALUE = {
    0: 1200,
    1: 2400,
    2: 4800,
    3: 9600,
    4: 19200,
    5: 38400,
}

BAUD_VALUE_TO_INDEX = {value: key for key, value in BAUD_INDEX_TO_VALUE.items()}

REG_CH_STATE = {1: 0x00, 2: 0x01}
REG_CH_MODE = {1: 0x02, 2: 0x03}
REG_CH_JOG_TIME = {1: 0x04, 2: 0x05}
REG_CH_DELAY_TIME = {1: 0x06, 2: 0x07}
REG_CH_DUTY = {1: 0x08, 2: 0x09}
REG_CH_FREQ = {1: 0x0B, 2: 0x0D}

REG_POWER_LOSS_MEMORY = 0x0E
REG_DEVICE_ADDRESS = 0x20
REG_DEVICE_BAUD = 0x21
REG_SPECIAL_COMMAND = 0xFF00


@dataclass
class SerialConfig:
    port: str
    serial_baudrate: int
    parity: str
    stopbits: int
    bytesize: int
    timeout: float
    slave: int


def build_client(config: SerialConfig) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=config.port,
        baudrate=config.serial_baudrate,
        parity=config.parity,
        stopbits=config.stopbits,
        bytesize=config.bytesize,
        timeout=config.timeout,
    )


def run_modbus(config: SerialConfig, action):
    client = build_client(config)
    if not client.connect():
        raise RuntimeError(f"Unable to open serial port: {config.port}")
    try:
        return action(client)
    finally:
        client.close()


def write_reg(config: SerialConfig, register: int, value: int) -> None:
    def _action(client: ModbusSerialClient):
        result = client.write_register(address=register, value=value, device_id=config.slave)  # type: ignore[call-arg]
        if result.isError():
            raise RuntimeError(f"Write failed. register=0x{register:04X}, value={value}")

    run_modbus(config, _action)


def read_reg(config: SerialConfig, register: int) -> int:
    def _action(client: ModbusSerialClient):
        result = client.read_holding_registers(address=register, count=1, device_id=config.slave)  # type: ignore[call-arg]
        if result.isError():
            raise RuntimeError(f"Read failed. register=0x{register:04X}")
        return int(result.registers[0])

    return run_modbus(config, _action)


def time_to_reg_value(seconds: float) -> int:
    value = int(round(seconds * 10))
    return max(1, min(65535, value))


def reg_value_to_time(value: int) -> float:
    return round(value / 10.0, 1)


class ChannelPanel(QWidget):
    def __init__(self, channel: int, get_config, log_callback):
        super().__init__()
        self.channel = channel
        self.get_config = get_config
        self.log = log_callback
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)

        form = QFormLayout()
        self.mode = QComboBox()
        self.mode.addItems(list(MODE_LABEL_TO_VALUE.keys()))

        self.jog_time = QDoubleSpinBox()
        self.jog_time.setRange(0.1, 6553.5)
        self.jog_time.setSingleStep(0.1)
        self.jog_time.setValue(1.0)

        self.delay_time = QDoubleSpinBox()
        self.delay_time.setRange(0.1, 6553.5)
        self.delay_time.setSingleStep(0.1)
        self.delay_time.setValue(1.0)

        self.duty = QSpinBox()
        self.duty.setRange(0, 100)
        self.duty.setValue(50)

        self.freq = QSpinBox()
        self.freq.setRange(1, 1000)
        self.freq.setValue(100)

        form.addRow(f"CH{self.channel} 模式", self.mode)
        form.addRow(f"CH{self.channel} 点动时间 (s)", self.jog_time)
        form.addRow(f"CH{self.channel} 延时开启时间 (s)", self.delay_time)
        form.addRow(f"CH{self.channel} PWM 占空比 (%)", self.duty)
        form.addRow(f"CH{self.channel} PWM 频率 (Hz)", self.freq)
        layout.addLayout(form)

        button_row = QHBoxLayout()
        on_btn = QPushButton(f"CH{self.channel} 打开")
        off_btn = QPushButton(f"CH{self.channel} 关闭")
        write_btn = QPushButton(f"写入 CH{self.channel} 参数")
        read_btn = QPushButton(f"读取 CH{self.channel}")

        on_btn.clicked.connect(self.turn_on)
        off_btn.clicked.connect(self.turn_off)
        write_btn.clicked.connect(self.write_params)
        read_btn.clicked.connect(self.read_params)

        button_row.addWidget(on_btn)
        button_row.addWidget(off_btn)
        button_row.addWidget(write_btn)
        button_row.addWidget(read_btn)
        layout.addLayout(button_row)

    def turn_on(self) -> None:
        self._safe_action(lambda cfg: write_reg(cfg, REG_CH_STATE[self.channel], 1), f"CH{self.channel} 已打开")

    def turn_off(self) -> None:
        self._safe_action(lambda cfg: write_reg(cfg, REG_CH_STATE[self.channel], 0), f"CH{self.channel} 已关闭")

    def write_params(self) -> None:
        def _action(cfg):
            write_reg(cfg, REG_CH_MODE[self.channel], MODE_LABEL_TO_VALUE[self.mode.currentText()])
            write_reg(cfg, REG_CH_JOG_TIME[self.channel], time_to_reg_value(float(self.jog_time.value())))
            write_reg(cfg, REG_CH_DELAY_TIME[self.channel], time_to_reg_value(float(self.delay_time.value())))
            write_reg(cfg, REG_CH_DUTY[self.channel], int(self.duty.value()))
            write_reg(cfg, REG_CH_FREQ[self.channel], int(self.freq.value()))

        self._safe_action(_action, f"CH{self.channel} 参数写入成功")

    def read_params(self) -> None:
        def _action(cfg):
            state_value = read_reg(cfg, REG_CH_STATE[self.channel])
            mode_value = read_reg(cfg, REG_CH_MODE[self.channel])
            jog_value = read_reg(cfg, REG_CH_JOG_TIME[self.channel])
            delay_value = read_reg(cfg, REG_CH_DELAY_TIME[self.channel])
            duty_value = read_reg(cfg, REG_CH_DUTY[self.channel])
            freq_value = read_reg(cfg, REG_CH_FREQ[self.channel])
            return (
                f"CH{self.channel}: 状态={state_value}, "
                f"模式={MODE_VALUE_TO_LABEL.get(mode_value, mode_value)}, "
                f"点动={reg_value_to_time(jog_value)}s, "
                f"延时={reg_value_to_time(delay_value)}s, "
                f"占空比={duty_value}%, 频率={freq_value}Hz"
            )

        self._safe_action(_action, success_message=None)

    def _safe_action(self, action, success_message=None) -> None:
        try:
            result = action(self.get_config())
            if isinstance(result, str):
                self.log(result)
            elif success_message:
                self.log(success_message)
        except Exception as exc:
            self.log(f"错误: {exc}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("QX-DO02 Modbus RTU 控制面板 (Qt)")
        self.resize(1200, 820)
        self._build_ui()

    def _build_ui(self) -> None:
        root = QWidget()
        root_layout = QVBoxLayout(root)

        conn_group = QGroupBox("连接设置")
        conn_form = QGridLayout(conn_group)

        self.port = QLineEdit("COM3")
        self.serial_baud = QComboBox()
        for b in [1200, 2400, 4800, 9600, 19200, 38400]:
            self.serial_baud.addItem(str(b), b)
        self.serial_baud.setCurrentText("9600")

        self.parity = QComboBox()
        self.parity.addItems(["N", "E", "O"])

        self.stopbits = QComboBox()
        self.stopbits.addItems(["1", "2"])

        self.timeout = QDoubleSpinBox()
        self.timeout.setRange(0.1, 5.0)
        self.timeout.setSingleStep(0.1)
        self.timeout.setValue(0.5)

        self.slave = QSpinBox()
        self.slave.setRange(1, 255)
        self.slave.setValue(1)

        test_btn = QPushButton("连接测试（读取地址寄存器）")
        test_btn.clicked.connect(self.connection_test)

        conn_form.addWidget(QLabel("串口"), 0, 0)
        conn_form.addWidget(self.port, 0, 1)
        conn_form.addWidget(QLabel("串口波特率"), 0, 2)
        conn_form.addWidget(self.serial_baud, 0, 3)
        conn_form.addWidget(QLabel("校验位"), 1, 0)
        conn_form.addWidget(self.parity, 1, 1)
        conn_form.addWidget(QLabel("停止位"), 1, 2)
        conn_form.addWidget(self.stopbits, 1, 3)
        conn_form.addWidget(QLabel("超时 (s)"), 2, 0)
        conn_form.addWidget(self.timeout, 2, 1)
        conn_form.addWidget(QLabel("从站地址"), 2, 2)
        conn_form.addWidget(self.slave, 2, 3)
        conn_form.addWidget(test_btn, 3, 0, 1, 4)

        root_layout.addWidget(conn_group)

        tabs = QTabWidget()
        tabs.addTab(self._build_channels_tab(), "通道控制")
        tabs.addTab(self._build_device_tab(), "设备设置")
        tabs.addTab(self._build_advanced_tab(), "高级")
        root_layout.addWidget(tabs)

        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        root_layout.addWidget(log_group)

        self.setCentralWidget(root)

    def _build_channels_tab(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)

        ch1_box = QGroupBox("通道 1")
        ch1_layout = QVBoxLayout(ch1_box)
        ch1_layout.addWidget(ChannelPanel(1, self.get_serial_config, self.log))

        ch2_box = QGroupBox("通道 2")
        ch2_layout = QVBoxLayout(ch2_box)
        ch2_layout.addWidget(ChannelPanel(2, self.get_serial_config, self.log))

        layout.addWidget(ch1_box)
        layout.addWidget(ch2_box)
        return widget

    def _build_device_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)

        cfg_box = QGroupBox("设备配置")
        cfg_layout = QFormLayout(cfg_box)
        self.enable_memory = QCheckBox("启用掉电记忆")
        self.enable_memory.setChecked(True)

        self.device_addr = QSpinBox()
        self.device_addr.setRange(1, 255)
        self.device_addr.setValue(1)

        self.device_baud = QComboBox()
        for value in BAUD_VALUE_TO_INDEX.keys():
            self.device_baud.addItem(str(value), value)
        self.device_baud.setCurrentText("9600")

        cfg_layout.addRow(self.enable_memory)
        cfg_layout.addRow("设备地址", self.device_addr)
        cfg_layout.addRow("设备波特率", self.device_baud)

        btn_row = QHBoxLayout()
        mem_btn = QPushButton("写入掉电记忆")
        addr_btn = QPushButton("写入设备地址")
        baud_btn = QPushButton("写入设备波特率")
        read_btn = QPushButton("读取设备配置")

        mem_btn.clicked.connect(self.write_memory_setting)
        addr_btn.clicked.connect(self.write_device_address)
        baud_btn.clicked.connect(self.write_device_baud)
        read_btn.clicked.connect(self.read_device_config)

        btn_row.addWidget(mem_btn)
        btn_row.addWidget(addr_btn)
        btn_row.addWidget(baud_btn)
        btn_row.addWidget(read_btn)

        special_box = QGroupBox("特殊操作")
        special_layout = QHBoxLayout(special_box)
        reboot_btn = QPushButton("重启设备")
        reset_btn = QPushButton("恢复出厂并重启")
        reboot_btn.clicked.connect(self.reboot_device)
        reset_btn.clicked.connect(self.factory_reset)
        special_layout.addWidget(reboot_btn)
        special_layout.addWidget(reset_btn)

        layout.addWidget(cfg_box)
        layout.addLayout(btn_row)
        layout.addWidget(special_box)
        layout.addStretch()
        return widget

    def _build_advanced_tab(self) -> QWidget:
        widget = QWidget()
        layout = QFormLayout(widget)

        self.raw_addr = QSpinBox()
        self.raw_addr.setRange(0, 65535)
        self.raw_value = QSpinBox()
        self.raw_value.setRange(0, 65535)

        btn_row = QHBoxLayout()
        read_btn = QPushButton("读取寄存器")
        write_btn = QPushButton("写入寄存器")
        read_btn.clicked.connect(self.read_raw_register)
        write_btn.clicked.connect(self.write_raw_register)
        btn_row.addWidget(read_btn)
        btn_row.addWidget(write_btn)

        layout.addRow("寄存器地址 (十进制)", self.raw_addr)
        layout.addRow("值", self.raw_value)
        layout.addRow(btn_row)
        return widget

    def get_serial_config(self) -> SerialConfig:
        return SerialConfig(
            port=self.port.text().strip(),
            serial_baudrate=int(self.serial_baud.currentData()),
            parity=self.parity.currentText(),
            stopbits=int(self.stopbits.currentText()),
            bytesize=8,
            timeout=float(self.timeout.value()),
            slave=int(self.slave.value()),
        )

    def log(self, msg: str) -> None:
        self.log_view.append(msg)

    def _safe_action(self, action, ok_message: str | None = None) -> None:
        try:
            result = action(self.get_serial_config())
            if isinstance(result, str):
                self.log(result)
            elif ok_message:
                self.log(ok_message)
        except Exception as exc:
            self.log(f"错误: {exc}")
            QMessageBox.critical(self, "Modbus 错误", str(exc))

    def connection_test(self) -> None:
        def _action(cfg):
            value = read_reg(cfg, REG_DEVICE_ADDRESS)
            return f"连接成功。设备地址寄存器 = {value}"

        self._safe_action(_action)

    def write_memory_setting(self) -> None:
        self._safe_action(
            lambda cfg: write_reg(cfg, REG_POWER_LOSS_MEMORY, 1 if self.enable_memory.isChecked() else 0),
            "掉电记忆写入成功",
        )

    def write_device_address(self) -> None:
        self._safe_action(
            lambda cfg: write_reg(cfg, REG_DEVICE_ADDRESS, int(self.device_addr.value())),
            "设备地址写入成功",
        )

    def write_device_baud(self) -> None:
        def _action(cfg):
            baud_value = int(self.device_baud.currentData())
            write_reg(cfg, REG_DEVICE_BAUD, BAUD_VALUE_TO_INDEX[baud_value])

        self._safe_action(_action, "设备波特率写入成功")

    def read_device_config(self) -> None:
        def _action(cfg):
            memory_value = read_reg(cfg, REG_POWER_LOSS_MEMORY)
            addr_value = read_reg(cfg, REG_DEVICE_ADDRESS)
            baud_idx = read_reg(cfg, REG_DEVICE_BAUD)
            return f"掉电记忆={memory_value}, 地址={addr_value}, 波特率={BAUD_INDEX_TO_VALUE.get(baud_idx, baud_idx)}"

        self._safe_action(_action)

    def reboot_device(self) -> None:
        self._safe_action(lambda cfg: write_reg(cfg, REG_SPECIAL_COMMAND, 0x0001), "已发送重启命令")

    def factory_reset(self) -> None:
        self._safe_action(lambda cfg: write_reg(cfg, REG_SPECIAL_COMMAND, 0xFFFF), "已发送恢复出厂命令")

    def read_raw_register(self) -> None:
        def _action(cfg):
            addr = int(self.raw_addr.value())
            value = read_reg(cfg, addr)
            return f"0x{addr:04X} = {value} (0x{value:04X})"

        self._safe_action(_action)

    def write_raw_register(self) -> None:
        def _action(cfg):
            addr = int(self.raw_addr.value())
            val = int(self.raw_value.value())
            write_reg(cfg, addr, val)

        self._safe_action(_action, f"已写入 {int(self.raw_value.value())} 到 0x{int(self.raw_addr.value()):04X}")


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("QX-DO02 Qt 控制器")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
