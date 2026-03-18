# LED-485-055 (RS485) 控制脚本

基于说明书 ASCII 协议：`$001,内容#`

已实现功能：

- 日期与时间交替显示，每 2 秒切换一次
	- 日期格式：`MM-DD`，例如 `03-21`
	- 时间格式：`HH.MM`，例如 `14.30`
- 秒倒计时（2 位或 3 位数字都支持）
- 倒计时结束后闪动显示 `EOF`

## 安装依赖

```bash
pip install -e .
```

## 运行

0) 打开 GUI（推荐）

```bash
python main.py --gui
```

也可以直接运行（无参数默认打开 GUI）：

```bash
python main.py
```

GUI 支持：

- 串口选择、地址/波特率、位数
- 模式切换（日期时间/倒计时/搅拌器联动）
- 开始/停止、运行日志
- 在搅拌器联动模式下，直接控制搅拌器：
	- 下发转速
	- 下发温度
	- 停止搅拌+加热

新增：搅拌器联动模式（双串口）

- LED 数码管使用一个串口（例如 `COM3`）
- 磁力搅拌器使用另一个串口（例如 `COM5`）
- 程序会轮询搅拌器状态，并在数码管上每 2 秒切换显示：
	- 温度：`t` + 数值（例如 `t  25`）
	- 转速：`5` + 数值（用 `5` 代替 `s`，例如 `51200`）

命令行示例：

```bash
python main.py --port COM3 --mode stirrer --stir-port COM5 --stir-baudrate 9600 --digits 5 --switch-seconds 2 --poll-seconds 0.5
```

1) 日期/时间交替显示（055 默认 5 位）

```bash
python main.py --port COM3 --address 1 --baudrate 9600 --mode clock --digits 5
```

2) 从 88 秒倒计时，结束后 EOF 闪烁

```bash
python main.py --port COM3 --address 1 --baudrate 9600 --mode countdown --countdown-start 88 --digits 3
```

对于 055（5 位），也可以使用 `--digits 5`，倒计时数字会自动右对齐显示。

## 参数说明

- `--port`：串口号，例如 `COM3`
- `--address`：屏地址，默认 `1`（协议中会格式化为 `001`）
- `--baudrate`：默认 `9600`
- `--mode`：`clock`、`countdown` 或 `stirrer`
- `--countdown-start`：倒计时起始秒数，默认 `88`
- `--digits`：屏位数，055 用 `5`
- `--stir-port`：搅拌器串口（`stirrer` 模式必填）
- `--stir-baudrate`：搅拌器波特率，默认 `9600`
- `--poll-seconds`：搅拌器状态采样间隔，默认 `0.5`
