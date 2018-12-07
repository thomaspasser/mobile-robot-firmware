#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Odometry.h"
#include "Motors.h"

typedef struct {
  double current;
  double angular_velocity;
  double angle;
} MotorState;

#define CONTROLLER_DT 1000 // 1 ms

class MotorController{
  public:
    MotorController(Motors * motors, Odometry * odom);
    void reset();
    void update();
    void setReferenceVelocity(double vref_l, double vref_r);
    double vLeft();
    double vRight();

  private:
    Odometry * _odom;
    Motors * _motors;
    long lastUpdateTime;
    double F11, F12, F21, F22, F31, F32;
    double G1, G2, G3;
    double C3;
    double L1, L2, L3;

    double vref_l, vref_r;
    double Kc1, Kc2;
    double N;

    MotorState leftMotor, rightMotor;
    double yhatLeft;
    double yhatRight;
};

#endif
