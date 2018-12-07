#ifndef ROBOT_DEFINITIONS_H
#define ROBOT_DEFINITIONS_H

// Physical parameters
// All units in cm
// Front wheel
#define WHEELBASE 13.5

#define SERVO_MIN_ANGLE 8.0
#define SERVO_MAX_ANGLE 172.0
#define SERVO_MID 88
#define SERVO_PIN 7

// Main dimensions
#define TRACK 17.0

// Back wheels, motors
#define WHEEL_RADIUS 3.0
#define GEARING 75.81
#define CPR 12

#define PWM_PERIOD 6250 // hundredth of usecs (1e-8 secs) -> 16 kHz

#define LEFT_MOTOR_SPEED_PIN 8
#define RIGHT_MOTOR_SPEED_PIN 9
#define LEFT_MOTOR_DIR_PIN 10
#define RIGHT_MOTOR_DIR_PIN 11

#define REVERSE_LEFT_MOTOR 0
#define REVERSE_RIGHT_MOTOR 1

#define REVERSE_LEFT_ENCODER 0
#define REVERSE_RIGHT_ENCODER 1

#define MOTOR_DRIVER_MODE_PIN 13

#define MOTOR_MAX_VOLTAGE 6.0
#define MOTOR_MIN_VOLTAGE 0.0

// Battery
#define MIN_BATTERY_VOLTAGE 7

#define BATTERY_VOLTAGE_PIN 8
#define BATTERY_RATIO 2.78132348

// Encoders
#define LEFT_ENCODER_PIN1 5
#define LEFT_ENCODER_PIN2 4

#define RIGHT_ENCODER_PIN1 3
#define RIGHT_ENCODER_PIN2 2

// Line sensor
#define LINESENSOR_DETECTOR_SPACING 1.0
#define LINESENSOR_THRESHOLD 35.0
#define LINE_WIDTH 3.8

// Logging
#define LOG 1
#define LOGGING_DT 1000

// Command parser parameters
#define NUM_COMMAND_TYPES 4
#define NUM_STOP_TYPES 4

enum robotState {WAIT, IN_MISSION, CALIBW, CALIBB, MANUAL};

typedef struct {
  double leftVoltage;
  double rightVoltage;
  double leftVelocity;
  double rightVelocity;
  bool velocityControl;
} rcParams;

/*
MISSION PARSER
Command, commandvalue
- RECKON, speed (sign is direction)
- FOLLOW, side of line (speed?)
- TURN, speed
- TURNRADIUS, speed

With only one command parameter I need to fix some values
Maybe always have speed as a parameter?
Or have speed as a command? like set speed

Stoptype, parameter
 - DIST, distance at which to stop
 - ANGLE, (used with turn)
 - CROSSING_LINE, 0/1 side at which to look for crossing line (what about for both sides? 2?)
 - FRONT_DIST, distance; stop if distance sensor reading is smaller than this parameter
*/

enum commandType { RECKON=0, FOLLOW, TURN, TURNRADIUS};
enum stopType { DIST=0, ANGLE, CROSSING_LINE, FRONT_DIST};

typedef struct {
  enum commandType command;
  int commandValue;
  enum stopType stopCondition;
  int stopValue;
} MissionStep;

#endif
