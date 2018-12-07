#ifndef MOTORS_H
#define MOTORS_H

#include "definitions.h"
#include "pwm_lib.h"

using namespace arduino_due::pwm_lib;

class Motors{
  public:
    Motors();
    void init();
    void setVoltageRight(double voltage);
    void setVoltageLeft(double voltage);
    double batteryVoltage;
    double getVoltageLeft();
    double getVoltageRight();
    void disable();
    void enable();
  private:
    double voltageLeft;
    double voltageRight;
    bool disabled;
    pwm<pwm_pin::PWML5_PC22> pwmLeft; // pins are now defined here..
    pwm<pwm_pin::PWML4_PC21> pwmRight;
    int percentToDuty(float percent);
};

#endif
