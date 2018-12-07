#include "Motors.h"

#include "Arduino.h"

// Constructor
Motors::Motors(){
  batteryVoltage = 8;
  disabled = true;
}

void Motors::init(){
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);

  // pwm_lib init
  pwmLeft.start(PWM_PERIOD, 0);
  pwmRight.start(PWM_PERIOD, 0);

  // Set DRV8835 mode
  pinMode(MOTOR_DRIVER_MODE_PIN, OUTPUT);
  digitalWrite(MOTOR_DRIVER_MODE_PIN, HIGH);
}

void Motors::disable(){
  //analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
  //analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
  pwmLeft.set_duty(0);
  voltageLeft = 0.0;
  pwmRight.set_duty(0);
  voltageRight = 0.0;
  disabled = true;
}

void Motors::enable(){
  disabled = false;
}

void Motors::setVoltageRight(double voltage){
  if(!disabled && batteryVoltage > MIN_BATTERY_VOLTAGE){

    bool reverse = REVERSE_RIGHT_MOTOR;
      if(voltage < 0){
        reverse = !reverse;
        voltage = -voltage;
      }

      // limit output
      if(voltage > MOTOR_MAX_VOLTAGE) voltage = MOTOR_MAX_VOLTAGE;

      //int value = voltage/batteryVoltage * 255;
      float percent = voltage/batteryVoltage;
      pwmRight.set_duty(percentToDuty(percent));
      //analogWrite(RIGHT_MOTOR_SPEED_PIN, value);

      // set direction
      if(reverse){
        digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);
      } else {
        digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
      }

      voltageRight = voltage;
    } else {
      //Serial.println("Low battery voltage");
    }
}

void Motors::setVoltageLeft(double voltage){
  if(!disabled && batteryVoltage > MIN_BATTERY_VOLTAGE){

    bool reverse = REVERSE_LEFT_MOTOR;
      if(voltage < 0){
        reverse = !reverse;
        voltage = -voltage;
      }

      if(voltage > MOTOR_MAX_VOLTAGE) voltage = MOTOR_MAX_VOLTAGE;

      //int value = voltage/batteryVoltage * 255;
      float percent = voltage/batteryVoltage;
      pwmLeft.set_duty(percentToDuty(percent));

      //analogWrite(LEFT_MOTOR_SPEED_PIN, value);

      // set direction
      if(reverse){
        digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH);
      } else {
        digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
      }

      voltageLeft = voltage;
    } else {
      //Serial1.println("Low battery voltage");
    }
}

double Motors::getVoltageLeft(){
  return voltageLeft;
}

double Motors::getVoltageRight(){
  return voltageRight;
}

int Motors::percentToDuty(float percent){
  return percent * PWM_PERIOD;
}
