#include "modbusReMap.h"
#include "motor.h"
#include <Arduino.h>

Motor motor1(3, 4, 400.0f, 0); // 夹紧电机，占用 EEPROM 0-3 字节
Motor motor2(5, 6, 400.0f, 4); // 放卷电机，占用 EEPROM 4-7 字节
Motor motor3(7, 8, 400.0f, 8); // 顶升电机，占用 EEPROM 8-11 字节

Motor* motors[] = {&motor1, &motor2, &motor3};
const uint8_t motorCount = sizeof(motors) / sizeof(motors[0]);

void setup()
{
    motor1.loadPositionFromEEPROM();
    motor2.loadPositionFromEEPROM();
    motor3.loadPositionFromEEPROM();

    setupModbus(DeRePin(2), 115200L);
    setupMotorMap(motors, motorCount);
}

void loop()
{
    getModbus().task();
    runMotors(); // 必须以最高频率执行步进脉冲生成，确保运动平滑

    // 限制参数解析与状态回传的频率（如 20ms 一次 = 50Hz），避免高频浮点数运算拖慢 MCU
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    if (now - lastUpdate >= 20) {
        lastUpdate = now;
        handleMotorControl();
        updateMotorStatus();
    }
}
