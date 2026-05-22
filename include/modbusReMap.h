#ifndef MODBUS_REMAP_H
#define MODBUS_REMAP_H

#include "motor.h"
#include <ModbusRTU.h>

#define REG_PER_MOTOR 10 // 每个电机所需保持寄存器数量
#define START_ADDR 0     // Modbus 起始地址

extern ModbusRTU mb;

void setupModbus(uint8_t DeRePin, int baudRate = 9600);
void setupMotorMap(Motor *motors[], uint8_t count);
void updateMotorStatus();
void handleMotorControl();

#endif
