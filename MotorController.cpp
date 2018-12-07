#include "MotorController.h"
#include "definitions.h"

MotorController::MotorController(Motors * motors, Odometry * odom){
  _odom = odom;
  _motors = motors;
  lastUpdateTime = 0;

  F11 = 0.99365706;
	F12 = -0.01150749;
	F21 = 0.01594945;
	F22 = 0.99990784;
	F31 = 0.00000798;
	F32 = 0.00099997;
	F11 = 0.99365706;
	F12 = -0.01150749;
	F21 = 0.01594945;
	F22 = 0.99990784;
	F31 = 0.00000798;
	F32 = 0.00099997;

  G1 = 5.34221167;
	G2 = 0.04278302;
	G3 = 0.00001427;

  C3 = CPR/(2*M_PI);

  L1 = -0.00004754;
	L2 = 0.00002715;
	L3 = 0.00016861;

  Kc1 = 0.011713292301403;
  Kc2 = 0.019601126618802;
  N = 0.021755195;

  leftMotor = {0};
  rightMotor = {0};

  yhatLeft = 0;
  yhatRight = 0;
}

void MotorController::reset(){
  leftMotor = {0};
  rightMotor = {0};

  yhatLeft = 0;
  yhatRight = 0;
}

void MotorController::setReferenceVelocity(double vref_l, double vref_r){
  this->vref_l = vref_l*GEARING/WHEEL_RADIUS;
  this->vref_r = vref_r*GEARING/WHEEL_RADIUS;
}

void MotorController::update(){
  long now = micros();
  if(now-lastUpdateTime >= CONTROLLER_DT){
    // Observer
    double yleft = _odom->getLeftCounts();
    double yright = _odom->getRightCounts();

    double leftVoltage = _motors->getVoltageLeft();
    double rightVoltage = _motors->getVoltageRight();

    MotorState newStateLeft;
    newStateLeft.current = F11*leftMotor.current + F12*leftMotor.angular_velocity + G1*leftVoltage + L1*(yleft-yhatLeft);
    newStateLeft.angular_velocity = F21*leftMotor.current + F22*leftMotor.angular_velocity + G2*leftVoltage + L2*(yleft-yhatLeft);
    newStateLeft.angle = leftMotor.angle + F31*leftMotor.current + F32*leftMotor.angular_velocity + G3*leftVoltage +L3*(yleft-yhatLeft);

    yhatLeft = C3*newStateLeft.angle;
    leftMotor = newStateLeft;

    MotorState newStateRight;
    newStateRight.current = F11*rightMotor.current + F12*rightMotor.angular_velocity + G1*rightVoltage + L1*(yright-yhatRight);
    newStateRight.angular_velocity = F21*rightMotor.current + F22*rightMotor.angular_velocity + G2*rightVoltage + L2*(yright-yhatRight);
    newStateRight.angle = rightMotor.angle + F31*rightMotor.current + F32*rightMotor.angular_velocity + G3*rightVoltage +L3*(yright-yhatRight);

    yhatRight = C3*newStateRight.angle;
    rightMotor = newStateRight;

    // Full state feedback with reference tracking
    double newLeftVoltage = N*vref_l - (Kc1*leftMotor.current + Kc2*leftMotor.angular_velocity);
    double newRightVoltage = N*vref_r - (Kc1*rightMotor.current + Kc2*rightMotor.angular_velocity);
    _motors->setVoltageLeft(newLeftVoltage);
    _motors->setVoltageRight(newRightVoltage);

    lastUpdateTime += CONTROLLER_DT;
  }
}

double MotorController::vLeft(){
  return leftMotor.angular_velocity*WHEEL_RADIUS/GEARING;
}

double MotorController::vRight(){
  return rightMotor.angular_velocity*WHEEL_RADIUS/GEARING;
}
