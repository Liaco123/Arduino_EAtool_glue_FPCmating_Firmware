#ifndef MODBUS_REMAP_H
#define MODBUS_REMAP_H

#include <ModbusRTU.h>

#include "motor.h"


#define REG_PER_MOTOR 10  // 每个电机所需保持寄存器数量
#define START_ADDR 0      // Modbus 起始地址

struct ModbusAddr {
  int addr;
  explicit ModbusAddr(int a) : addr(a) {}
};

struct DeRePin {
  uint8_t pin;
  explicit DeRePin(uint8_t p) : pin(p) {}
};

ModbusRTU& getModbus();

void setupModbus(DeRePin deRePin, int baudRate = 9600);
void setupMotorMap(Motor* motors[], uint8_t count);
void updateMotorStatus();
void handleMotorControl();

#endif
