#include <SoftwareSerial.h>

// 传感器接 D2(RX), D3(TX)
SoftwareSerial laserSerial(2, 3);

void setup() {
  Serial.begin(9600);       // 给电脑串口助手输出
  laserSerial.begin(9600);  // 传感器通信
  Serial.println("传感器已启动");
}

void loop() {
  // ==============================
  // 官方手册 读取距离指令（9字节）
  // ==============================
  byte readCmd[] = {0x62, 0x32, 0x09, 0x00, 0x00, 0x00, 0x01, 0x00, 0x5A};
  laserSerial.write(readCmd, sizeof(readCmd));

  delay(100);

  // 读取传感器返回的 9 字节数据
  if (laserSerial.available() >= 9) {
    byte buf[9];
    laserSerial.readBytes(buf, 9);

    // ==============================
    // 官方正确距离计算公式
    // 距离 = 第4字节 ×256 + 第5字节
    // ==============================
    int distance = buf[4] * 256 + buf[5];

    // 输出到 PC 串口助手
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

  delay(200);
}