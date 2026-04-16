# QX-DO02 Control UI

This project provides a PySide6 (Qt) desktop UI to control the QX-DO02 PWM signal generator relay module over Modbus RTU (RS485).

## Features

- Serial connection setup (COM port, baud, parity, stop bits, timeout, slave id)
- Channel 1 and Channel 2 control
	- ON/OFF
	- Mode: normal / jog / delay_on / pwm
	- Jog time and delay-on time
	- PWM duty and frequency
	- Read back channel status
- Device settings
	- Power-loss memory
	- Device address
	- Device baud register
	- Reboot / factory reset command
- Raw register read/write tool

## Install

Use your preferred package workflow. Example with pip:

```bash
pip install -e .
```

## Run

```bash
python main.py
```

This will open a native Qt desktop window.

## Register map used

- CH1/CH2 state: 0x00 / 0x01
- CH1/CH2 mode: 0x02 / 0x03
- CH1/CH2 jog time: 0x04 / 0x05 (0.1 s units)
- CH1/CH2 delay-on time: 0x06 / 0x07 (0.1 s units)
- CH1/CH2 PWM duty: 0x08 / 0x09
- CH1/CH2 PWM freq: 0x0B / 0x0D
- Power-loss memory: 0x0E
- Device address: 0x20
- Device baud index: 0x21
- Special command: 0xFF00
