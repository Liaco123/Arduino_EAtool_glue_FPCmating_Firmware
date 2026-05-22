#include "modbusReMap.h"
#include "motor.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

Motor motor1(3, 4, 1600.0f, 0);
Motor motor2(5, 6, 400.0f, 2);
Motor motor3(7, 8, 1600.0f, 4);

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
  mb.begin(&sfSerial, 2);
  mb.setBaudrate(9600);
  mb.server(1);
  setupMotorMap(motors, motorCount);
}

void loop() {
  mb.task();
  handleMotorControl();
  updateMotorStatus();
}
