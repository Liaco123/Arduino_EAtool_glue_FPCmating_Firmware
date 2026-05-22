#ifndef MOTOR_H
#define MOTOR_H

#include <AccelStepper.h>
#include <EEPROM.h>
#include <stdint.h>

#define STATUS_ENABLED 0x01   // bit 0
#define STATUS_RUNNING 0x02   // bit 1
#define STATUS_COMPLETED 0x04 // bit 2
#define STATUS_ERROR 0x08     // bit 3

class Motor : public AccelStepper {
private:
  float _stepsPerUnit;     // 每单位的步数（用于单位转换）
  float _maxSpeedUnit;     // 单位速度
  float _accelerationUnit; // 单位加速度
  float _targetPos;
  int _eepromAddr; // EEPROM 地址
  bool _enabled;   // 当前使能状态
  bool _errorFlag; // 错误状态标志
  bool _runed;
  float _lastPos;
  bool _completed;
  bool _running;

public:
  Motor(uint8_t stepPin, uint8_t dirPin, float stepsPerUnit = 100.0f,
        int eepromAddr = 0);

  // 获取参数
  float getCurrentPos();       // 当前单位位置
  float getTargetPos();        // 目标单位位置
  float getMaxSpeedUnit();     // 单位速度
  float getAccelerationUnit(); // 单位加速度

  // 设置参数
  void setTargetPos(float pos);          // 设置目标单位位置
  bool setCurrentPos(float pos);         // 设置当前位置为单位值
  void setMaxSpeedUnit(float speed);     // 设置单位速度
  void setAccelerationUnit(float accel); // 设置单位加速度

  // 初始化 & EEPROM
  void initial();                // 设置当前位置为 0
  void savePositionToEEPROM();   // 保存当前位置到 EEPROM
  void loadPositionFromEEPROM(); // 从 EEPROM 加载位置

  // 急停
  void emergencyStop(); // 急停，停止运动，保持当前位置

  // CiA 402 简化控制
  void handleControlWord(uint16_t controlWord); // 控制字解析
  uint16_t getStatusWord();                     // 获取当前状态字
  void setError(bool error);                    // 设置错误标志

  void disable();         // 取消使能
  void enable();          // （可选）重新使能
  bool isEnabled() const; // 返回当前是否已使能
};

#endif // MOTOR_H
