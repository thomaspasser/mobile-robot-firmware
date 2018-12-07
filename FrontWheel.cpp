#include "FrontWheel.h"
#include "definitions.h"

#include "Arduino.h"

FrontWheel::FrontWheel(){
  angle = 0;
}

void FrontWheel::init(){
  _servo.attach(SERVO_PIN);
  _servo.write(SERVO_MID);
}

void FrontWheel::setAngle(double angle){
  double beta = SERVO_MID + angle;
  if(beta > SERVO_MAX_ANGLE) beta = SERVO_MAX_ANGLE;
  else if(beta < SERVO_MIN_ANGLE) beta = SERVO_MIN_ANGLE;

  _servo.write(beta);
}

double FrontWheel::turnRadiusToAngle(double radius){
  // Radius in cm,  angle in radians
  return atan(WHEELBASE/radius)/PI*180.0;
}

double FrontWheel::turnRadiusToDv(double vel, double radius){
  // Get dv from desired turn radius
  // dv is half of v2-v1
  return 0.5*TRACK * vel/radius;
}

double FrontWheel::dvToTurnRadius(double vel, double dv){
  return 0.5 * TRACK * vel/dv;
}

double FrontWheel::angularVelocityToDv(double omega){
  // Get dv from desired angular velocity
  return omega * 2 * TRACK;
}

double FrontWheel::dvToAngle(double dv, double vel){
  // Angle of caster given some dv
  // Used when following line
  return turnRadiusToAngle( dvToTurnRadius(vel,dv) );
}
