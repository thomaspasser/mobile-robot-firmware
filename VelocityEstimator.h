#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

#include "Odometry.h"

#define VELOCITY_CALC_DT 10000 // 10 ms

class VelocityEstimator{
  public:
    VelocityEstimator(Odometry * odom);
    void calculate();
    double vLeft();
    double vRight();
    void debug();

  private:
    Odometry * _odom;
    long lastCalcTime;
    long lastLeftCounts;
    long lastRightCounts;
    double vleft;
    double vright;
    double cm_per_count;
};

#endif
