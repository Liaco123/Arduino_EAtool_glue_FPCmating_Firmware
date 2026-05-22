#include "modbusReMap.h"

ModbusRTU mb;
ModbusRTU& getModbus() {
  return mb;
}
static Motor** motorArray = nullptr;
static uint8_t motorCount = 0;

constexpr int SERVER_ID = 1;

// Modbus 每个电机寄存器偏移（相对于每个电机的 base 地址）
enum MotorRegisterOffset : uint8_t {
  CONTROL_WORD = 0,
  STATUS_WORD = 1,
  TARGET_POS_H = 2,
  TARGET_POS_L = 3,
  SPEED_H = 4,
  SPEED_L = 5,
  ACCEL_H = 6,
  ACCEL_L = 7,
  CUR_POS_H = 8,
  CUR_POS_L = 9
};

// 联合体：float 与 uint16_t[2] 之间转换
union FloatUint16 {
  float f;
  uint16_t u16[2];  // 注意索引 0 是低位，1 是高位（小端）
};

// 将 float 写入大端序两个寄存器（高字节先）
inline void writeFloatToModbus(float value, ModbusAddr baseAddr) {
  FloatUint16 data;
  data.f = value;
  getModbus().Hreg(baseAddr.addr, data.u16[1]);      // 高 16 位
  getModbus().Hreg(baseAddr.addr + 1, data.u16[0]);  // 低 16 位
}

// 从两个大端序寄存器读取 float（高字节在前）
inline float readFloatFromModbus(ModbusAddr baseAddr) {
  FloatUint16 data;
  data.u16[1] = getModbus().Hreg(baseAddr.addr);      // 高 16 位
  data.u16[0] = getModbus().Hreg(baseAddr.addr + 1);  // 低 16 位
  return data.f;
}

void setupModbus(DeRePin deRePin, int baudRate) {
  Serial.begin(baudRate, SERIAL_8N1);
  getModbus().begin(&Serial, deRePin.pin);
  getModbus().setBaudrate(baudRate);
  getModbus().server(SERVER_ID);
}

void setupMotorMap(Motor* motors[], uint8_t count) {
  motorArray = motors;
  motorCount = count;

  getModbus().addHreg(START_ADDR, 0, motorCount * REG_PER_MOTOR);
}

void updateMotorStatus() {
  if (!motorArray) return;

  for (uint8_t i = 0; i < motorCount; i++) {
    int base = START_ADDR + i * REG_PER_MOTOR;
    Motor* m = motorArray[i];

    // 状态字
    getModbus().Hreg(base + STATUS_WORD, m->getStatusWord());

    // 当前位置信息（float → 2 reg）
    writeFloatToModbus(m->getCurrentPos(), ModbusAddr(base + CUR_POS_H));
  }
}

void handleMotorControl() {
  if (!motorArray) return;

  for (uint8_t i = 0; i < motorCount; i++) {
    int base = START_ADDR + i * REG_PER_MOTOR;
    Motor* m = motorArray[i];

    // 目标位置、速度、加速度
    m->setTargetPos(readFloatFromModbus(ModbusAddr(base + TARGET_POS_H)));
    m->setMaxSpeedUnit(readFloatFromModbus(ModbusAddr(base + SPEED_H)));
    m->setAccelerationUnit(readFloatFromModbus(ModbusAddr(base + ACCEL_H)));

    // 控制字
    uint16_t controlWord = getModbus().Hreg(base + CONTROL_WORD);
    m->handleControlWord(controlWord);
    m->run();
  }
}
