#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include "VelocityEstimator.h"
#include "Motors.h"

#define CONTROLLER_DT 5000

class VelocityController{
  public:
    VelocityController(Motors * motors, VelocityEstimator * estimator);
    void update();
    void setReferenceVelocity(double vref_l, double vref_r);
    void reset();

  private:
    Motors * _motors;
    VelocityEstimator * _estimator;
    double vref_l;
    double vref_r;
    long lastUpdateTime;
    double eSumLeft;
    double eSumRight;
    double maxSum;
    double kff;
    double kp;
    double ki;
};

#endif
