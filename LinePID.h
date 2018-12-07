#ifndef LINEPID_H
#define LINEPID_H

#include "Arduino.h"
#include "definitions.h"

#define LINE_PID_PERIOD 20000 // 50 Hz

class LinePID{
  public:
    LinePID();
    double update(double linePos);
  private:
    double dv;

    double Kp;
    double Ki;
    double Kd;
    double lastLinePos;
    double SampleTimeInSec;
    long lastUpdateTime;
};

#endif
