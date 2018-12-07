#ifndef FRONTWHEEL_H
#define FRONTWHEEL_H

#include <Servo.h>

class FrontWheel{
  public:
    FrontWheel();
    void init();
    void setAngle(double angle);
    double turnRadiusToAngle(double radius);
    double turnRadiusToDv(double vel, double radius);
    double dvToTurnRadius(double vel, double dv);
    double angularVelocityToDv(double omega);
    double dvToAngle(double dv, double vel);

  private:
    Servo _servo;
    double angle;
};

#endif

