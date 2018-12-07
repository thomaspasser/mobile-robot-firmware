#include "LineSensor.h"

LineSensor::LineSensor(HardwareSerial *serial) :
pins{A7, A6, A5, A4, A3, A2, A1, A0}
{
  _serial = serial;
  lastUpdateTime = 0;

  linePos = 0.0;
  leftEdge = 0.0;
  rightEdge = 0.0;

  for(unsigned int i = 0; i < 8; i++){
    black[i] = 0;
    white[i] = 0;
  }

  blackCalibrated = false;
  whiteCalibrated = false;
}

void LineSensor::init(){

  byte magic = dueFlashStorage.read(0);
  if(magic == 42){
    _serial->println("Calibration values found in flash!");
    // read from flash if there is something saved
    byte* b = dueFlashStorage.readAddress(4);
    memcpy(black, b, sizeof(int)*8);

    b = dueFlashStorage.readAddress(36);
    memcpy(white, b, sizeof(int)*8);

    whiteCalibrated = true;
    blackCalibrated = true;
  }
}

void LineSensor::update(){
  long now = micros();
  if(now-lastUpdateTime >= LINESENSOR_DT){
    // Read values, normalize between 0 and 100 with calibraton
    for(int i = 0; i < 8; i++){
      int raw = analogRead(pins[i]);

      if (raw > white[i]){
        val[i] = 100;
      } else if(raw < black[i]){
        val[i] = 0;
      } else {
        val[i] = (raw-black[i]) / ((double)(white[i]-black[i])) * 100.0;
      }
    }

    // now calculate line position
    // Try to find line edges

    bool foundLeft = false;
    unsigned int firstLeft;
    for(unsigned int i = 0; i < 8; i++){
      if(val[i] > LINESENSOR_THRESHOLD){
        firstLeft = i;
        foundLeft = true;
        break;
      }
    }

    bool foundRight = false;
    unsigned int firstRight;
    for(unsigned int i = 7; i >= 0; i--){
      if(val[i] > LINESENSOR_THRESHOLD){
        firstRight = i;
        foundRight = true;
        break;
      }
    }

    if(foundLeft && foundRight){
      double leftEdgeCalc;
      if(firstLeft == 0){
        leftEdgeCalc = 0;
      } else {
        leftEdgeCalc = ((firstLeft-1) + (LINESENSOR_THRESHOLD-val[firstLeft-1])/(val[firstLeft]-val[firstLeft-1]))*LINESENSOR_DETECTOR_SPACING;
      }

      double rightEdgeCalc;
      if(firstRight == 7){
        rightEdgeCalc = 7*LINESENSOR_DETECTOR_SPACING;
      } else {
        rightEdgeCalc = (firstRight + (LINESENSOR_THRESHOLD-val[firstRight])/(val[firstRight+1]-val[firstRight]))*LINESENSOR_DETECTOR_SPACING;
      }

      // Shift coordinates, zero in the middle instead of at sensor 0
      leftEdgeCalc = leftEdgeCalc-3.5;
      rightEdgeCalc = rightEdgeCalc-3.5;

      // Middle of line is between edges
      linePos = 0.5*(leftEdgeCalc+rightEdgeCalc);
      // Adjust to known line width
      leftEdge  = linePos - 0.5*LINE_WIDTH;
      rightEdge = linePos + 0.5*LINE_WIDTH;
    } else {
      // If no line found, keep last values. Might turn back on line? :)
      //linePos = 0;
      //leftEdge  = linePos - 0.5*LINE_WIDTH;
      //rightEdge = linePos + 0.5*LINE_WIDTH;
    }

    //log();

    lastUpdateTime += LINESENSOR_DT;
  }
}

double LineSensor::middle(){
  return linePos;
}

double LineSensor::left(){
  return leftEdge;
}

double LineSensor::right(){
  return rightEdge;
}

void LineSensor::log(){
  _serial->print("LSR ");
  for(unsigned int i = 0; i < 8; i++){
    _serial->print(val[i]);
    _serial->print(" ");
  }
  _serial->println();

  _serial->print("LSP ");
  _serial->print(leftEdge);
  _serial->print(" ");
  _serial->print(linePos);
  _serial->print(" ");
  _serial->print(rightEdge);
  _serial->println();
}

void LineSensor::calibrateBlack(){
  _serial->println("Calibrating black..");
  for(int i = 0; i < 8; i++){
    double sum = 0.0;
    for(int j = 0; j < 1000; j++){
      sum += analogRead(pins[i]);
      delay(1);
    }
    double mean = sum/1000.0;
    _serial->print(mean);
    _serial->print(" ");
    black[i] = 0.9*mean;
  }
  _serial->println("");

  // Save in "flash"
  byte b[sizeof(int)*8];
  memcpy(b, black, sizeof(int)*8);
  dueFlashStorage.write(4, b, sizeof(int)*8);

  blackCalibrated = true;
  if(whiteCalibrated){
    dueFlashStorage.write(0,42);
  }
}

void LineSensor::calibrateWhite(){
  _serial->println("Calibrating white..");
  for(int i = 0; i < 8; i++){
    double sum = 0.0;
    for(int j = 0; j < 1000; j++){
      sum += analogRead(pins[i]);
      delay(1);
    }
    double mean = sum/1000.0;
    _serial->print(mean);
    _serial->print(" ");
    white[i] = 1.1*mean;
  }
  _serial->println("");

  // Save in flash
  byte b[sizeof(int)*8];
  memcpy(b, white, sizeof(int)*8);
  dueFlashStorage.write(36, b, sizeof(int)*8);

  whiteCalibrated = true;

  if(blackCalibrated){
    dueFlashStorage.write(0,42);
  }
}
