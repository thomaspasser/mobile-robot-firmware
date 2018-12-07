#include "VelocityEstimator.h"
#include "definitions.h"

VelocityEstimator::VelocityEstimator(Odometry * odom){
  _odom = odom;
  lastLeftCounts = 0;
  lastRightCounts = 0;
  lastCalcTime = 0;
  cm_per_count = 2.0*PI*WHEEL_RADIUS/(double)(CPR*GEARING);
}

void VelocityEstimator::debug(){
  //Serial1.print("CM_PER_COUNT: ");
  //Serial1.println(cm_per_count);
}

void VelocityEstimator::calculate(){
  long now = micros();
  if(now-lastCalcTime >= VELOCITY_CALC_DT){
    long leftCounts = _odom->getLeftCounts();
    long rightCounts = _odom->getRightCounts();

    // Simple estimator..
    // Just for now ? :o)

    //Serial1.print("Counts difference: ");
    //Serial1.println(cm_per_count*(leftCounts-lastLeftCounts));

    vleft  = cm_per_count*(double)(leftCounts-lastLeftCounts)  /(double)VELOCITY_CALC_DT * 1000000;
    vright = cm_per_count*(double)(rightCounts-lastRightCounts)/(double)VELOCITY_CALC_DT * 1000000;

    //Serial1.print("vleft: ");
    //Serial1.println(vleft);

    lastLeftCounts = leftCounts;
    lastRightCounts = rightCounts;

    lastCalcTime += VELOCITY_CALC_DT;
  }
}

double VelocityEstimator::vLeft(){
  return vleft;
}

double VelocityEstimator::vRight(){
  return vright;
}
