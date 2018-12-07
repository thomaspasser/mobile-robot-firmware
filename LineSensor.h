#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "Arduino.h"
#include <DueFlashStorage.h>

#include "definitions.h"

#define LINESENSOR_DT 1000

class LineSensor{
  public:
    LineSensor(HardwareSerial *serial);
    void init();
    void update();
    double middle();
    double left();
    double right();
    void calibrateBlack();
    void calibrateWhite();
    void log();

  private:
    HardwareSerial * _serial;
    int pins[8];
    double val[8];
    long lastUpdateTime;
    double linePos;
    double leftEdge;
    double rightEdge;

    DueFlashStorage dueFlashStorage;
    int black[8];
    int white[8];
    bool whiteCalibrated;
    bool blackCalibrated;
};

#endif
