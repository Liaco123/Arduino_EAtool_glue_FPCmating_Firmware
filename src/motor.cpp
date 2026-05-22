#include "motor.h"
#include <Arduino.h>
#include <math.h>

Motor::Motor(uint8_t stepPin, uint8_t dirPin, float stepsPerUnit,
             int eepromAddr)
    : AccelStepper(AccelStepper::DRIVER, stepPin, dirPin),
      _stepsPerUnit(stepsPerUnit), _eepromAddr(eepromAddr), _enabled(false),
      _errorFlag(false) {
  float pos = 0.0f;
  EEPROM.get(_eepromAddr, pos);

  if (isnan(pos) || fabs(pos) > 100000.0f) {
    setCurrentPosition(0); // EEPROM 无效，初始化为 0
  } else {
    setCurrentPosition(pos * _stepsPerUnit);
  }
}

float Motor::getCurrentPos() { return currentPosition() / _stepsPerUnit; }

float Motor::getTargetPos() { return _targetPos / _stepsPerUnit; }

float Motor::getMaxSpeedUnit() { return maxSpeed() / _stepsPerUnit; }

float Motor::getAccelerationUnit() { return acceleration() / _stepsPerUnit; }

void Motor::setTargetPos(float pos) {
  _targetPos = pos * _stepsPerUnit;
  // Serial.print("_targetPos = ");
  // Serial.println(_targetPos);
}

bool Motor::setCurrentPos(float pos) {
  setCurrentPosition(pos * _stepsPerUnit);
  return true;
}

void Motor::setMaxSpeedUnit(float speed) {
  _maxSpeedUnit = speed;
  // Serial.print("speed = ");
  // Serial.println(speed * _stepsPerUnit);
  setMaxSpeed(speed * _stepsPerUnit);
}

void Motor::setAccelerationUnit(float accel) {
  _accelerationUnit = accel;
  // Serial.print("accel = ");
  // Serial.println(accel * _stepsPerUnit);
  setAcceleration(accel * _stepsPerUnit);
}

void Motor::initial() { setCurrentPosition(0); }

void Motor::savePositionToEEPROM() {
  float pos = getCurrentPos();
  EEPROM.put(_eepromAddr, pos);
}

void Motor::loadPositionFromEEPROM() {
  float pos = 0.0f;
  EEPROM.get(_eepromAddr, pos);
  if (!isnan(pos)) {
    setCurrentPos(pos);
  } else {
    setCurrentPosition(0);
  }
}

void Motor::emergencyStop() {
  stop();                                // 停止运动（非急停）
  setCurrentPosition(currentPosition()); // 锁定当前位置
  _enabled = false;
}

void Motor::handleControlWord(uint16_t cw) {
  if (!(cw & 0x01)) { // bit 0: 使能
    emergencyStop();
    return;
  }

  enable();

  if (cw & 0x10) { // bit 4: 急停（新定义）
    emergencyStop();
    return;
  }

  if (cw & 0x02) { // bit 1: 运动触发
    if (!_running) {

      if (cw & 0x04) { // bit 2: 绝对位置
        moveTo(_targetPos);

      } else {
        move(_targetPos);
      }
      _lastPos = _targetPos;
    }
  }
}

uint16_t Motor::getStatusWord() {
  uint16_t status = 0;

  if (_enabled)
    status |= STATUS_ENABLED;

  _running = isRunning();
  if (_running) {
    status |= STATUS_RUNNING;
    _runed = true;
  } else {
    if (_runed && distanceToGo() == 0) {
      status |= STATUS_COMPLETED;
      _runed = false;
    }
  }

  if (_errorFlag)
    status |= STATUS_ERROR;

  return status;
}

void Motor::setError(bool error) { _errorFlag = error; }

void Motor::disable() { _enabled = false; }

void Motor::enable() { _enabled = true; }

bool Motor::isEnabled() const { return _enabled; }
