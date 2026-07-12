#pragma once
#include <Arduino.h>

// lifecycle
void settingsLoad();
void settingsSave();
void settingsDefaults();
void settingsPrint();

// throttle config
extern bool invertThrottle1;
extern bool invertThrottle2;
extern uint16_t throttleMaxDiff;

// gear tuning
extern float upshiftGain;
extern float downshiftGain;
extern bool throttleCalMode;
extern bool throttleCalWaitingMax;

extern uint16_t learnedT1Min;
extern uint16_t learnedT1Max;

extern uint16_t learnedT2Min;
extern uint16_t learnedT2Max;
