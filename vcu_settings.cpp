#include "vcu_settings.h"
#include <EEPROM.h>

// EEPROM layout
#define ADDR_T1INV          0
#define ADDR_T2INV          1

#define ADDR_THROTTLE_DIFF  2

#define ADDR_T1MIN          4
#define ADDR_T1MAX          6
#define ADDR_T2MIN          8
#define ADDR_T2MAX         10

#define ADDR_GEAR_GAIN     20


// extern variables from main
extern bool invertThrottle1;
extern bool invertThrottle2;
extern uint16_t throttleMaxDiff;
extern float gearGain[6];
extern uint16_t throttle1Min;
extern uint16_t throttle1Max;

extern uint16_t throttle2Min;
extern uint16_t throttle2Max;
bool throttleCalMode = false;
bool throttleCalWaitingMax = false;

uint16_t learnedT1Min = 0;
uint16_t learnedT1Max = 0;

uint16_t learnedT2Min = 0;
uint16_t learnedT2Max = 0;


// --------------------------------------------------
// LOAD
// --------------------------------------------------
void settingsLoad()
{
  invertThrottle1 = EEPROM.read(ADDR_T1INV) != 0;
  invertThrottle2 = EEPROM.read(ADDR_T2INV) != 0;

  EEPROM.get(ADDR_THROTTLE_DIFF, throttleMaxDiff);
  EEPROM.get(ADDR_T1MIN, throttle1Min);
EEPROM.get(ADDR_T1MAX, throttle1Max);

EEPROM.get(ADDR_T2MIN, throttle2Min);
EEPROM.get(ADDR_T2MAX, throttle2Max);

  for (int i = 1; i <= 5; i++)
  {
    EEPROM.get(ADDR_GEAR_GAIN + (i * sizeof(float)), gearGain[i]);
  }

  Serial.println("Settings loaded");
}

// --------------------------------------------------
// SAVE
// --------------------------------------------------
void settingsSave()
{
  EEPROM.update(ADDR_T1INV, invertThrottle1);
  EEPROM.update(ADDR_T2INV, invertThrottle2);

  EEPROM.put(ADDR_THROTTLE_DIFF, throttleMaxDiff);
  EEPROM.put(ADDR_T1MIN, throttle1Min);
EEPROM.put(ADDR_T1MAX, throttle1Max);

EEPROM.put(ADDR_T2MIN, throttle2Min);
EEPROM.put(ADDR_T2MAX, throttle2Max);

  for (int i = 1; i <= 5; i++)
  {
    EEPROM.put(ADDR_GEAR_GAIN + (i * sizeof(float)), gearGain[i]);
  }

  Serial.println("Settings saved");
}

// --------------------------------------------------
// DEFAULTS
// --------------------------------------------------
void settingsDefaults()
{
  invertThrottle1 = false ;
  invertThrottle2 = false;

  throttleMaxDiff = 300;
  throttle1Min = 323;
throttle1Max = 795;

throttle2Min = 211;
throttle2Max = 683;

  gearGain[1] = 1.00;
  gearGain[2] = 1.00;
  gearGain[3] = 1.00;
  gearGain[4] = 1.00;
  gearGain[5] = 1.00;

  Serial.println("Defaults loaded");
}

// --------------------------------------------------
// PRINT
// --------------------------------------------------

void settingsPrint()
{
  Serial.println("---- SETTINGS ----");

  Serial.print("T1 invert: ");
  Serial.println(invertThrottle1);

  Serial.print("T2 invert: ");
  Serial.println(invertThrottle2);

  Serial.print("Throttle diff max: ");
  Serial.println(throttleMaxDiff);

  // -------------------------
  // Throttle calibration
  // -------------------------
  Serial.print("T1 Min: ");
  Serial.println(throttle1Min);

  Serial.print("T1 Max: ");
  Serial.println(throttle1Max);

  Serial.print("T2 Min: ");
  Serial.println(throttle2Min);

  Serial.print("T2 Max: ");
  Serial.println(throttle2Max);

  Serial.println("Gear gains:");

  for (int i = 1; i <= 5; i++)
  {
      Serial.print("Gear ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(gearGain[i], 3);
  }

  Serial.println("------------------");
}
