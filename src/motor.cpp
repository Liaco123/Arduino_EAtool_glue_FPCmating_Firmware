#include "motor.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

Motor::Motor(uint8_t stepPin, uint8_t dirPin, float stepsPerUnit,
             int eepromAddr)
    : AccelStepper(AccelStepper::DRIVER, stepPin, dirPin),
      _stepsPerUnit(stepsPerUnit),
      _maxSpeedUnit(0.0f),
      _accelerationUnit(0.0f),
      _targetPos(0.0f),
      _eepromAddr(eepromAddr),
      _enabled(false),
      _errorFlag(false),
      _runed(false),
      _lastPos(0.0f),
      _completed(false),
      _running(false),
      _lastTriggerState(false),
      _lastInitState(false) {
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
  if (!isnan(pos) && fabs(pos) <= 100000.0f) {
    setCurrentPos(pos);
  } else {
    setCurrentPosition(0);
  }
}

void Motor::emergencyStop() {
  stop();                                // 减速并停止运动
  setCurrentPosition(currentPosition()); // 锁定当前物理位置
  _enabled = false;
  _runed = false;                        // 重置运行标记以防止误判触发
  savePositionToEEPROM();                // 急停时立刻持久化当前位置
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

  // bit 3: 初始化 (上升沿检测)
  bool currentInit = (cw & 0x08) != 0;
  if (currentInit && !_lastInitState) {
    initial();
    savePositionToEEPROM(); // 初始化为 0 后立刻持久化
  }
  _lastInitState = currentInit;

  // bit 1: 运动触发 (上升沿检测)
  bool currentTrigger = (cw & 0x02) != 0;
  if (currentTrigger && !_lastTriggerState) {
    if (!_running) {
      if (cw & 0x04) { // bit 2: 绝对位置
        moveTo(_targetPos);
      } else {
        move(_targetPos);
      }
      _lastPos = _targetPos;
    }
  }
  _lastTriggerState = currentTrigger;
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
      savePositionToEEPROM(); // 运动完成到达目标位置时，自动持久化当前位置到 EEPROM
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
