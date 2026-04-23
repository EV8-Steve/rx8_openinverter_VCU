#include "vcu_settings.h"
#include <EEPROM.h>

// EEPROM layout
#define ADDR_T1INV        0
#define ADDR_T2INV        1
#define ADDR_THROTTLE_DIFF 2   // 2 bytes
#define ADDR_GEAR_GAIN     10  // floats start here

// extern variables from main
extern bool invertThrottle1;
extern bool invertThrottle2;
extern uint16_t throttleMaxDiff;
extern float gearGain[6];

// --------------------------------------------------
// LOAD
// --------------------------------------------------
void settingsLoad()
{
  invertThrottle1 = EEPROM.read(ADDR_T1INV) != 0;
  invertThrottle2 = EEPROM.read(ADDR_T2INV) != 0;

  EEPROM.get(ADDR_THROTTLE_DIFF, throttleMaxDiff);

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
  invertThrottle1 = false;
  invertThrottle2 = true;

  throttleMaxDiff = 300;

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
