#include "Arduino.h"
#include "Odometry.h"
#include "definitions.h"

Odometry::Odometry(Encoder *left, Encoder *right){
  LeftEnc = left;
  RightEnc = right;

  leftCounts = 0;
  rightCounts = 0;

  radsPerCount = 2.0*PI/(CPR*GEARING);

  init();
}

// maybe we want to do this again during a mission?
void Odometry::init(){
  x = 0;
  y = 0;
  theta = 0;
  d = 0;
  x0 = 0;
  y0 = 0;
  theta0 = 0;
  d0 = 0;
}

void Odometry::read(){
  // Read new values
  double newLeftCounts = LeftEnc->read();
  double newRightCounts = RightEnc->read();

  if(REVERSE_LEFT_ENCODER) newLeftCounts   = -newLeftCounts;
  if(REVERSE_RIGHT_ENCODER) newRightCounts = -newRightCounts;

  // Calculate wheel rotations
  double leftPhi = 0;
  double rightPhi = 0;
  if(newLeftCounts != leftCounts) leftPhi = (newLeftCounts-leftCounts)*radsPerCount;
  if(newRightCounts != rightCounts) rightPhi = (newRightCounts-rightCounts)*radsPerCount;

  // Update odometry
  double dD = WHEEL_RADIUS/2.0 * (leftPhi + rightPhi);
  d += dD;
  // could add half dTheta to the angle here?
  x += cos(theta)*dD;
  y += sin(theta)*dD;
  theta += WHEEL_RADIUS/TRACK * (rightPhi-leftPhi);

  // Save values for next loop
  leftCounts = newLeftCounts;
  rightCounts = newRightCounts;
}

double Odometry::X(){
  return x;
}

double Odometry::Y(){
  return y;
}

double Odometry::Theta(){
  return theta*180.0/PI;
}

double Odometry::D(){
  return d;
}

double Odometry::r(){
  return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0));
}

double Odometry::relX(){
  return r()*cos(theta-theta0);
}

double Odometry::relY(){
  return r()*sin(theta-theta0);
}

double Odometry::relTheta(){
  return (theta-theta0)*180.0/PI;
}

double Odometry::relD(){
  return d-d0;
}

void Odometry::zero(){
  x0 = x;
  y0 = y;
  theta0 = theta;
  d0 = d;
}

long Odometry::getLeftCounts(){
  return leftCounts;
}

long Odometry::getRightCounts(){
  return rightCounts;
}
