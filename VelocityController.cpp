#include "VelocityController.h"

#include "Arduino.h"

VelocityController::VelocityController(Motors * motors, VelocityEstimator * estimator){
  _motors = motors;
  _estimator = estimator;

  vref_l = 0;
  vref_r = 0;

  eSumLeft = 0;
  eSumRight = 0;
  maxSum = 5;

  double SampleTimeInSec = (double)CONTROLLER_DT/1000000;
  kff = 0.05;
  kp = 0.05;
  ki = 0.00005/SampleTimeInSec;
}

void VelocityController::reset(){
  eSumLeft = 0;
  eSumRight = 0;
}

void VelocityController::update(){
  long now = micros();
  if(now-lastUpdateTime >= CONTROLLER_DT){
    double vLeft = _estimator->vLeft();
    double vRight = _estimator->vRight();

    double eLeft = vref_l - vLeft;
    double eRight = vref_r - vRight;
    //Serial1.print("Error: ");
    //Serial1.println(eLeft);

    eSumLeft += ki*eLeft;
    eSumRight += ki*eRight;
    //Serial1.print("Error int: ");
    //Serial1.println(eSumLeft);

    // Integration limit
    if(eSumLeft > maxSum) eSumLeft = maxSum;
    if(eSumRight > maxSum) eSumRight = maxSum;

    double voltageLeft = kff*vref_l + kp*eLeft + eSumLeft;
    double voltageRight = kff*vref_r + kp*eRight + eSumRight;
    //Serial1.print("Voltage: ");
    //Serial1.println(voltageLeft);

    _motors->setVoltageLeft(voltageLeft);
    _motors->setVoltageRight(voltageRight);

    lastUpdateTime += VELOCITY_CALC_DT;
  }
}

void VelocityController::setReferenceVelocity(double vref_l, double vref_r){
  //Serial1.print("Set reference velocity: ");
  //Serial1.println(vref_l);
  this->vref_l = vref_l;
  this->vref_r = vref_r;
  //Serial1.println("Controller parameters: ");
  //Serial1.println(kff);
  //Serial1.println(kp);
  //Serial1.println(ki);
}
