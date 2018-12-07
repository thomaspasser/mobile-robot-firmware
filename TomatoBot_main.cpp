#include "TomatoBot_main.h"

long now, lastLogTime;

rcParams remoteControl = {0};

robotState state = WAIT; // running or stopped
robotState lastState = WAIT;
MissionStep * mission;
int missionLine = -1;

double vref_l, vref_r;
Encoder LeftEnc(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
Encoder RightEnc(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);

Motors motors;
FrontWheel frontWheel;

Odometry odom(&LeftEnc,&RightEnc);
//VelocityEstimator velocityEstimator(&odom);
//VelocityController velocityController(&motors, &velocityEstimator);
MotorController motorController(&motors, &odom); // TODO test this controller

LinePID linePID;

long startTurnTime;
bool startedTurning;

LineSensor lineSensor(&Serial);
CommandParser commandParser(&Serial,&state,&remoteControl);

double followSpeed = 35.0;

void readSerial(){
  commandParser.read();
}

void readSensors(){
  odom.read();
  lineSensor.update();

  // Read battery voltage (filter?)
  motors.batteryVoltage = BATTERY_RATIO*analogRead(BATTERY_VOLTAGE_PIN)*(3.3/4095.0);
}

void calc(){
  //velocityEstimator.calculate();
}

void stop(){
  frontWheel.setAngle(0);
  motors.disable();
  motorController.reset();
  motorController.setReferenceVelocity(0,0);
  Serial.println("stopping...");
  state = WAIT;
}

void stepTransition(){
  if(missionLine > commandParser.missionSteps()-1){
    // mission is done
    missionLine = -1;
    stop();
  } else {

    // odom setpoint
    odom.zero();
    switch(mission[missionLine].command){
      case RECKON:
        frontWheel.setAngle(0);
        motorController.setReferenceVelocity(mission[missionLine].commandValue,mission[missionLine].commandValue);
        motors.enable();

        break;
      case FOLLOW:
        frontWheel.setAngle(0);
        motorController.setReferenceVelocity(followSpeed, followSpeed);
        motors.enable();
        break;

      case TURN:
        motors.disable();
        if(mission[missionLine].commandValue > 0){
          frontWheel.setAngle(-90);
        } else {
          frontWheel.setAngle(90);
        }
        //motorController.setReferenceVelocity(-mission[missionLine].commandValue,mission[missionLine].commandValue);
        //motors.enable(); wait with this
        startTurnTime = now;
        startedTurning = false;

        break;
      case TURNRADIUS:

        break;
    }
  }
}

void progressMission(){ // rename to something else, as we also do manual commands? (setvoltage etc.)

  switch(state){
    case WAIT:
      {
        if(commandParser.hasMission()){
          mission = commandParser.getMission();
        }

        if(missionLine >= 0){ // if was in mission before
          missionLine = -1;
        }

        if(lastState != WAIT){ // if was started in some way
          stop();
        }
      }
      break;

    case MANUAL:
      {

      }
      break;

    case IN_MISSION:
      {
        if(missionLine == -1) {
          // Mission just started (by serial START command), react on it
          missionLine = 0;
          stepTransition();
        }

        // Do stuff based on mission[missionLine].command
        switch(mission[missionLine].command){
          case RECKON:
            // Speed is set in stepTransition()

            break;
          case FOLLOW:
            {
            double linePos;
            if(mission[missionLine].commandValue == -1) linePos = lineSensor.left();
            else if(mission[missionLine].commandValue == 1) linePos = lineSensor.right();
            else linePos = lineSensor.middle();

            double dv = linePID.update(linePos);

            motorController.setReferenceVelocity(followSpeed+0.5*dv, followSpeed-0.5*dv);
            frontWheel.setAngle(frontWheel.dvToAngle(dv, followSpeed));

            break;
            }

          case TURN:
            // Wait a bit for turning motors on, to allow the front wheel to turn.
            if(!startedTurning && now-startTurnTime>500000){
              startedTurning = true;
              motorController.setReferenceVelocity(-mission[missionLine].commandValue,mission[missionLine].commandValue);
              motors.enable();
            }

            break;

          case TURNRADIUS:

            break;
        }

        // Decide if step is done based on mission[missionLine].stopCondition
        switch(mission[missionLine].stopCondition){
          case DIST:
            if(abs(odom.relD()) >= abs(mission[missionLine].stopValue)){
              missionLine++;
              stepTransition();
            }
            break;
          case ANGLE:
            if(abs(odom.relTheta()) >= abs(mission[missionLine].stopValue)){
              missionLine++;
              stepTransition();
            }
            break;

          case CROSSING_LINE:

            break;

          case FRONT_DIST:

            break;
        }

      }
      break;

    case CALIBW:
      {
        lineSensor.calibrateWhite(); // will hang for 8 seconds!
        Serial.println("ACK");
        state = WAIT;
      }
      break;

    case CALIBB:
      {
        lineSensor.calibrateBlack(); // will hang for 8 seconds!
        Serial.println("ACK");
        state = WAIT;
      }
      break;
  }

}

void updateMotors(){
  if(state == MANUAL){
    motors.enable();
    // TODO: Test remote control of velocity
    if(remoteControl.velocityControl){ // Velocity or voltage control
      motorController.setReferenceVelocity(remoteControl.leftVelocity, remoteControl.rightVelocity);
      motorController.update();
    } else {
      motors.setVoltageLeft(remoteControl.leftVoltage);
      motors.setVoltageRight(remoteControl.rightVoltage);
    }
  } else {
    motorController.update();
  }
}

void writeSerial(){
  // For logging. CommandParser also writes ACK/NACK sometimes
  #if LOG
  if(now-lastLogTime >= LOGGING_DT){
    Serial.print(now);
    Serial.print(" ");
    Serial.print(motors.batteryVoltage);
    Serial.print(" ");
    Serial.print(missionLine);
    Serial.print(" ");
    Serial.print(motors.getVoltageLeft(),4);
    Serial.print(" ");
    Serial.print(motors.getVoltageRight(),4);
    Serial.print(" ");
    Serial.print(odom.getLeftCounts());
    Serial.print(" ");
    Serial.print(odom.getRightCounts());
    Serial.print(" ");
    Serial.print(odom.X(),4);
    Serial.print(" ");
    Serial.print(odom.Y(),4);
    Serial.print(" ");
    Serial.print(odom.Theta(),4);
    Serial.print(" ");
    Serial.print(motorController.vLeft(),4);
    Serial.print(" ");
    Serial.print(motorController.vRight(),4); // test w ros code
    Serial.println(" ");
    //Serial.print("\n");
    lastLogTime += LOGGING_DT;
  }
  #endif
}



void setup() {
  Serial.begin(115200);
  Serial.println("Robot booted");
  analogReadResolution(12);
  motors.init();
  frontWheel.init();
  lineSensor.init();
  //velocityEstimator.debug();
  motorController.setReferenceVelocity(0,0);
}

void loop() {
  now = micros();
  readSerial();
  readSensors();
  calc();
  progressMission();
  updateMotors();
  writeSerial();
  lastState = state;
}
