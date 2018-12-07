#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Encoder.h>

class Odometry{
  public:
    Odometry(Encoder *left, Encoder *right);
    void read();
    double X();
    double Y();
    double D();
    double Theta();
    double relX();
    double relY();
    double relD();
    double relTheta();
    void init();
    void zero();
    long getLeftCounts();
    long getRightCounts();

  private:
    Encoder *LeftEnc;
    Encoder *RightEnc;
    long leftCounts;
    long rightCounts;
    double x,y,d,theta;
    double x0,y0,d0,theta0;
    double r();
    double radsPerCount;
};


#endif
