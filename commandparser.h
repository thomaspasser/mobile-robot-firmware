#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include "HardwareSerial.h"

#include "definitions.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_LINE_SIZE 100
#define MAX_MISSION_SIZE 2000

enum ParserState {IDLE, MISSIONLEN, MISSIONLINE};

class CommandParser{
  public:
    CommandParser(HardwareSerial *serial, robotState *state, rcParams *remote);
    int read();
    MissionStep * getMission();
    bool hasMission();
    unsigned int missionSteps();

  private:
    HardwareSerial * _serial;
    robotState * _robotState;
    rcParams * _remote;
    char buffer[MAX_LINE_SIZE];
    unsigned int index;
    char missionBuffer[MAX_MISSION_SIZE];
    unsigned int index2;
    unsigned int linesExpected;
    ParserState state;
    MissionStep *mission;
    bool _hasMission;
    unsigned int _missionSteps;

    const char *commandTypeString[NUM_COMMAND_TYPES];
    const char *stopTypeString[NUM_STOP_TYPES];

    int stringToCommandType(char *s);
    int stringToStopType(char *s);
    int parseMission(MissionStep ** mission);
};

#endif
