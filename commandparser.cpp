#include "commandparser.h"

// Constructor
CommandParser::CommandParser(HardwareSerial *serial, robotState *state, rcParams *remote):
commandTypeString{"RECKON", "FOLLOW", "TURN", "TURNRADIUS"}, stopTypeString{"DIST", "ANGLE", "CROSSING_LINE", "FRONT_DIST"}
{
  _serial = serial;
  _robotState = state;
  _remote = remote;

  _hasMission = false;
  this->state = IDLE;
  index = 0;
  index2 = 0;
}

int CommandParser::read(){

  int any = 0;

  int b = _serial->read();
  while(b != -1){
    any = 1;
    char s = (char)b;
    //_serial->print(b);

    if(s != '\n'){
      buffer[index] = s;
      index++;
    } else {
      //_serial->print("Detected newline or space, index: ");
      //_serial->println(index);
      if(index>0){

        char line[index+1];
        strncpy(line, buffer, index);
        line[index] = '\0'; // terminate C string, strncpy doesn't do this

        index = 0;

        // Finished reading line, do something with it
        // State machine
        switch(*_robotState){
          case WAIT:
            {
              switch(state){
                case IDLE:
                  if(strcmp(line,"MISSION") == 0){
                    state = MISSIONLEN;
                    // clear buffer, to be sure it's empty
                    memset(missionBuffer, 0 , sizeof(missionBuffer));

                    //_serial->println("Reading mission..");
                  } else if(strcmp(line,"START") == 0){
                    if(_hasMission){
                      *_robotState = IN_MISSION;
                      _serial->println("ACK");
                    } else {
                      _serial->println("NACK");
                    }

                  } else if(strcmp(line,"STOP") == 0){
                    *_robotState = WAIT;
                    _serial->println("ACK");
                  } else if(strcmp(line,"CALIBW") == 0){
                    *_robotState = CALIBW;
                  } else if(strcmp(line,"CALIBB") == 0){
                    *_robotState = CALIBB;
                  } else if(strcmp(line,"SETMODE REMOTE") == 0){ // switch to rc control mode
                    *_robotState = MANUAL;
                    *_remote = {0};
                    _serial->println("ACK");
                  } else {
                    _serial->println("NACK");
                  }
                  break;

                case MISSIONLEN:
                  {
                    char * l = strtok(line, " ");
                    char * cn = strtok(NULL, "\n");
                    if(strcmp(l,"LINES") == 0){
                      linesExpected = atoi(cn);
                      //_serial->print("Lines expected: ");
                      //_serial->println(linesExpected);
                      state = MISSIONLINE;
                    } else {
                      //Expected "LINES #", got something else
                      // SEND NACK, go back to idle
                      _serial->println("NACK");
                      state = IDLE;
                    }
                  }
                  break;

                case MISSIONLINE:
                  {
                    // If we get END, the whole mission has been recieved it, and we parse it.
                    if(strcmp(line,"END") == 0){
                      //_serial->println("got end");
                      state = IDLE;
                      // parse all mission lines, return OK or NOK
                      int res = parseMission(&mission);
                      if(res != -1){
                        unsigned int num_lines = res;
                        if(num_lines == linesExpected){
                          _serial->println("ACK");
                          _hasMission = true;
                          _missionSteps = num_lines;
                        } else {
                          // not expected number of lines
                          _serial->println("NACK");
                        }
                      } else {
                        //  parsing failed
                        _serial->println("NACK");
                      }

                    // If we don't get end, the line must be a mission line.
                    // We add it to the mission line buffer
                    } else {
                      //_serial->println("Got line");
                      strcat(missionBuffer, line);
                      strcat(missionBuffer, "\n"); // line is without the newline character
                    }
                  }
                  break;
              }
            }
            break;
          case IN_MISSION:
            {
              if(strcmp(line,"STOP") == 0){
                *_robotState = WAIT;
                _serial->println("ACK");
              } else {
                _serial->println("NACK");
              }
            }
            break;

          case MANUAL:
            {
              if(strcmp(line,"STOP") == 0){ // go out of remote control state
                *_robotState = WAIT;
                _serial->println("ACK");
              } else {
                // parse remote control cmd
                char * l = strtok(line, " ");
                char * cn = strtok(NULL, "\n");
                if(strcmp(l, "SETVOLTAGE") == 0){
                  char * sLeft = strtok(cn, " ");
                  char * sRight = strtok(NULL, "\n");
                  _remote->leftVoltage = atof(sLeft);
                  _remote->rightVoltage = atof(sRight);
                  _remote->velocityControl = false;
                  _serial->println("ACK");
                } else if(strcmp(l, "SETVELOCITY") == 0){
                  char * sLeft = strtok(cn, " ");
                  char * sRight = strtok(NULL, "\n");
                  _remote->leftVelocity = atof(sLeft);
                  _remote->rightVelocity = atof(sRight);
                  _remote->velocityControl = true;
                  _serial->println("ACK");
                } else {
                  _serial->println("NACK");
                }
              }

            }
            break;

        }

      }
    }

    b = _serial->read();
  }

}

int CommandParser::parseMission(MissionStep ** mission){
  //_serial->println(missionBuffer);
  // First count lines, then put each line in a char array, then parse each line
  int error = 0;

  // Count number of lines in mission
  unsigned int N = 0;
  const char *str;
  for(str = missionBuffer; *str; ++str){
    N += *str == '\n';
  }
  //printf("Found %i lines\n", N);

  char *mArray[N]; // array of pointers to char arrays

  // Allocate some memory for the mission data
  // calloc() zero-initializes the buffer, while malloc() leaves the memory uninitialized.
  // maybe just use malloc?
  *mission = (MissionStep*)calloc(N, sizeof(MissionStep));

  char *pch;
  pch = strtok(missionBuffer, "\r\n");

  unsigned int i = 0;
  while (pch != NULL){
    // Put each line into an array of char arrays
    mArray[i] = pch;
    //printf("Line %i: ", i);
    //printf("%s\n", pch);

    // put next line into pch
    pch = strtok(NULL,"\r\n");
    i++;
  }


  // Now I have an array of char arrays.. parse each
  for(unsigned int i = 0; i < N; i++){
    MissionStep mStep;

    //printf("Line %i: ", i);
    //printf("%s\n",mArray[i]);

    char* c = strtok(mArray[i], ",");
    char* s = strtok(NULL, ",");
    //printf("%s\n", c);
    //printf("%s\n", s);


    char* ct = strtok(c," ");
    char* cv = strtok(NULL, " ");
    //printf(">>%s<<\n", ct);

    // Get command
    // This returns -1 if no match.
    int res = stringToCommandType(ct);

    if(res == -1){
      error = 1;
      continue;
    }

    mStep.command = (commandType)res;

    // Get command value
    mStep.commandValue = atoi(cv);
    //printf("%i\n",mStep.commandValue);

    char* st = strtok(s," ");
    char* sv = strtok(NULL, " ");

    // Get stop condition
    res = stringToStopType(st);

    if(res == -1){
      error = 1;
      continue;
    }

    mStep.stopCondition = (stopType)res;

    mStep.stopValue = atoi(sv);
    //printf("%i\n", mStep.stopValue);



    // Push into my array
    (*mission)[i] = mStep;
  }

  if(error){
    return -1;
  } else {
    return N;
  }
}

MissionStep * CommandParser::getMission(){
  return mission;
}

bool CommandParser::hasMission(){
  return _hasMission;
}

unsigned int CommandParser::missionSteps(){
  return _missionSteps;
}

// Match with list of command types
// Returns -1 if no match
int CommandParser::stringToCommandType(char *s){
  for(unsigned int i=0; i<NUM_COMMAND_TYPES; i++){
    if(strcmp(s,commandTypeString[i]) == 0){
      return i;
    }
  }
  return -1;
}

int CommandParser::stringToStopType(char *s){
  for(unsigned int i=0; i<NUM_STOP_TYPES; i++){
    if(strcmp(s,stopTypeString[i]) == 0){
      return i;
    }
  }
  return -1;
}
