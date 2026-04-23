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
extern float gearGain[6];   // index 1–5 used
