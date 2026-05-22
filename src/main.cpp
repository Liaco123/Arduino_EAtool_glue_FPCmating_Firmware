#include "modbusReMap.h"
#include "motor.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

Motor motor1(3, 4, 1600.0f, 0);  // 占用 EEPROM 0-3 字节
Motor motor2(5, 6, 400.0f, 4);   // 修改为 4，占用 EEPROM 4-7 字节
Motor motor3(7, 8, 1600.0f, 8);  // 修改为 8，占用 EEPROM 8-11 字节

Motor *motors[] = {&motor1, &motor2, &motor3};
const uint8_t motorCount = sizeof(motors) / sizeof(motors[0]);
SoftwareSerial sfSerial(10, 11);

void setup() {
  motor1.loadPositionFromEEPROM();
  motor2.loadPositionFromEEPROM();
  motor3.loadPositionFromEEPROM();

  // setupModbus(2);
  sfSerial.begin(9600);
  Serial.begin(9600);
  getModbus().begin(&sfSerial, 2);
  getModbus().setBaudrate(9600);
  getModbus().server(1);
  setupMotorMap(motors, motorCount);
}

void loop() {
  getModbus().task();
  handleMotorControl();
  updateMotorStatus();
}
