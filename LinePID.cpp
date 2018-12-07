#include "LinePID.h"

LinePID::LinePID(){
  lastLinePos = 0.0;

  Kp = 2;
  Ki = 0;
  Kd = 0.2;

  SampleTimeInSec = (double)LINE_PID_PERIOD/1000000.0;
}

double LinePID::update(double linePos){
  long now = micros();
  // update value if the time has come
  if(now-lastUpdateTime >= LINE_PID_PERIOD){

    // I don't think we want integrator
    dv = Kp*linePos + Kd*(linePos-lastLinePos)/SampleTimeInSec;

    lastUpdateTime += LINE_PID_PERIOD;
  }
  return dv;
}
