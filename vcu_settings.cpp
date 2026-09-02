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

// --------------------------------------------------
// Gear gains
// --------------------------------------------------

#define ADDR_GEAR_GAIN          20


// --------------------------------------------------
// Shift torque ramps
// --------------------------------------------------

#define ADDR_TORQUE_RAMP_DOWN   40
#define ADDR_TORQUE_RAMP_UP     41


// --------------------------------------------------
// Traction Control PID
// --------------------------------------------------

// 5 x float = 20 bytes each
#define ADDR_TC_KP              50
#define ADDR_TC_KI              70
#define ADDR_TC_KD              90

// TC output slew limits
#define ADDR_TC_MAX_DOWN        110
#define ADDR_TC_MAX_UP          114


// --------------------------------------------------
// Settings validity
// --------------------------------------------------

#define ADDR_SETTINGS_MAGIC     12
#define SETTINGS_MAGIC           0x56435531UL


// extern variables from main
extern bool invertThrottle1;
extern bool invertThrottle2;
extern uint16_t throttleMaxDiff;

extern uint16_t throttle1Min;
extern uint16_t throttle1Max;

extern uint16_t throttle2Min;
extern uint16_t throttle2Max;

extern uint8_t SHIFT_TORQUE_RAMP_DOWN;
extern uint8_t SHIFT_TORQUE_RAMP_UP;

bool throttleCalMode = false;
bool throttleCalWaitingMax = false;

uint16_t learnedT1Min = 0;
uint16_t learnedT1Max = 0;

uint16_t learnedT2Min = 0;
uint16_t learnedT2Max = 0;

extern float gearGain[6];
extern float TC_KP[6];
extern float TC_KI[6];
extern float TC_KD[6];

extern float TC_MAX_TORQUE_DOWN_PER_SEC;
extern float TC_MAX_TORQUE_UP_PER_SEC;


bool settingsAreValid()
{
  uint32_t storedMagic = 0;

  EEPROM.get(ADDR_SETTINGS_MAGIC, storedMagic);

  return storedMagic == SETTINGS_MAGIC;
}
// --------------------------------------------------
// LOAD
// --------------------------------------------------
void settingsLoad()
{
  // Check that EEPROM contains settings written by this VCU code
  if (!settingsAreValid())
  {
    Serial.println("No valid EEPROM settings found");
    Serial.println("Loading safe defaults");

    settingsDefaults();
    settingsSave();

    return;
  }

// Load throttle settings
  invertThrottle1 = EEPROM.read(ADDR_T1INV) != 0;
  invertThrottle2 = EEPROM.read(ADDR_T2INV) != 0;

  EEPROM.get(ADDR_THROTTLE_DIFF, throttleMaxDiff);

  EEPROM.get(ADDR_T1MIN, throttle1Min);
  EEPROM.get(ADDR_T1MAX, throttle1Max);

  EEPROM.get(ADDR_T2MIN, throttle2Min);
  EEPROM.get(ADDR_T2MAX, throttle2Max);

//Load gear gains
  for (int i = 1; i <= 5; i++)
  {
    EEPROM.get(ADDR_GEAR_GAIN + ((i - 1) * sizeof(float)),
               gearGain[i]);

   if (gearGain[i] < 0.5f || gearGain[i] > 1.5f)
  gearGain[i] = 1.0f;
  }

// Load TC PID gains
  for (int i = 1; i <= 5; i++)
  {
    EEPROM.get(
      ADDR_TC_KP + ((i - 1) * sizeof(float)),
      TC_KP[i]
    );

    EEPROM.get(
      ADDR_TC_KI + ((i - 1) * sizeof(float)),
      TC_KI[i]
    );

    EEPROM.get(
      ADDR_TC_KD + ((i - 1) * sizeof(float)),
      TC_KD[i]
    );

    // Validate Kp
    if (TC_KP[i] < 0.0f || TC_KP[i] > 20.0f)
      TC_KP[i] = 2.0f;

    // Validate Ki
    if (TC_KI[i] < 0.0f || TC_KI[i] > 5.0f)
      TC_KI[i] = 0.20f;

    // Validate Kd
    if (TC_KD[i] < 0.0f || TC_KD[i] > 5.0f)
      TC_KD[i] = 0.05f;
  }

// Load TC output slew limits
  EEPROM.get(
    ADDR_TC_MAX_DOWN,
    TC_MAX_TORQUE_DOWN_PER_SEC
  );

  EEPROM.get(
    ADDR_TC_MAX_UP,
    TC_MAX_TORQUE_UP_PER_SEC
  );

  if (TC_MAX_TORQUE_DOWN_PER_SEC <= 0.0f ||
      TC_MAX_TORQUE_DOWN_PER_SEC > 1000.0f)
  {
    TC_MAX_TORQUE_DOWN_PER_SEC = 200.0f;
  }

  if (TC_MAX_TORQUE_UP_PER_SEC <= 0.0f ||
      TC_MAX_TORQUE_UP_PER_SEC > 1000.0f)
  {
    TC_MAX_TORQUE_UP_PER_SEC = 50.0f;
  }

//Load shift torque limits
  SHIFT_TORQUE_RAMP_DOWN = EEPROM.read(ADDR_TORQUE_RAMP_DOWN);
  SHIFT_TORQUE_RAMP_UP   = EEPROM.read(ADDR_TORQUE_RAMP_UP);

  if (SHIFT_TORQUE_RAMP_DOWN == 0 || SHIFT_TORQUE_RAMP_DOWN > 100)
    SHIFT_TORQUE_RAMP_DOWN = 20;

  if (SHIFT_TORQUE_RAMP_UP == 0 || SHIFT_TORQUE_RAMP_UP > 100)
    SHIFT_TORQUE_RAMP_UP = 10;


  Serial.println("Settings loaded");
}

// --------------------------------------------------
// SAVE
// --------------------------------------------------
void settingsSave()
{

//save throttle settings
  EEPROM.update(ADDR_T1INV, invertThrottle1);
  EEPROM.update(ADDR_T2INV, invertThrottle2);

  EEPROM.put(ADDR_THROTTLE_DIFF, throttleMaxDiff);
  EEPROM.put(ADDR_T1MIN, throttle1Min);
  EEPROM.put(ADDR_T1MAX, throttle1Max);

  EEPROM.put(ADDR_T2MIN, throttle2Min);
  EEPROM.put(ADDR_T2MAX, throttle2Max);


//Save gear gains
  for (int i = 1; i <= 5; i++)
  {
    EEPROM.put(ADDR_GEAR_GAIN + ((i - 1) * sizeof(float)),
               gearGain[i]);
  }

  // Save TC PID gains
  for (int i = 1; i <= 5; i++)
  {
    EEPROM.put(
      ADDR_TC_KP + ((i - 1) * sizeof(float)),
      TC_KP[i]
    );

    EEPROM.put(
      ADDR_TC_KI + ((i - 1) * sizeof(float)),
      TC_KI[i]
    );

    EEPROM.put(
      ADDR_TC_KD + ((i - 1) * sizeof(float)),
      TC_KD[i]
    );
  }

  // Save TC output slew limits
   EEPROM.put(
    ADDR_TC_MAX_DOWN,
    TC_MAX_TORQUE_DOWN_PER_SEC
  );

  EEPROM.put(
    ADDR_TC_MAX_UP,
    TC_MAX_TORQUE_UP_PER_SEC
  );

//Save shift torque limits
  EEPROM.update(ADDR_TORQUE_RAMP_DOWN, SHIFT_TORQUE_RAMP_DOWN);
  EEPROM.update(ADDR_TORQUE_RAMP_UP, SHIFT_TORQUE_RAMP_UP);

  EEPROM.put(ADDR_SETTINGS_MAGIC, SETTINGS_MAGIC);
  Serial.println("Settings saved");
}

// --------------------------------------------------
// DEFAULTS
// --------------------------------------------------
void settingsDefaults()
{

//throttle defaults  
  invertThrottle1 = false ;
  invertThrottle2 = false;

  throttleMaxDiff = 300;
  throttle1Min = 323;
  throttle1Max = 795;

  throttle2Min = 211;
  throttle2Max = 683;

//gear gain defaults  
  for (int i = 1; i <= 5; i++)
  {
    gearGain[i] = 1.000f;
  }

//shift torque limit defaults
  SHIFT_TORQUE_RAMP_DOWN = 20;
  SHIFT_TORQUE_RAMP_UP = 10;
 
//TC PID defaults
  for (int i = 1; i <= 5; i++)
  {
    TC_KP[i] = 2.0f;
    TC_KI[i] = 0.20f;
    TC_KD[i] = 0.05f;
  }

  TC_MAX_TORQUE_DOWN_PER_SEC = 200.0f;
  TC_MAX_TORQUE_UP_PER_SEC   = 100.0f;

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

  Serial.println("Gear Gains:");

  for (int i = 1; i <= 5; i++)
  {
    Serial.print("Gear ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(gearGain[i], 3);
  }

  Serial.print("Torque Ramp Down: ");
  Serial.println(SHIFT_TORQUE_RAMP_DOWN);

  Serial.print("Torque Ramp Up: ");
  Serial.println(SHIFT_TORQUE_RAMP_UP);

    Serial.println();

  Serial.println("Traction Control PID:");

  for (int i = 1; i <= 5; i++)
  {
    Serial.print("Gear ");
    Serial.print(i);

    Serial.print(" Kp=");
    Serial.print(TC_KP[i], 3);

    Serial.print(" Ki=");
    Serial.print(TC_KI[i], 3);

    Serial.print(" Kd=");
    Serial.println(TC_KD[i], 3);
  }

  Serial.print("TC Max Torque Down: ");
  Serial.print(TC_MAX_TORQUE_DOWN_PER_SEC, 1);
  Serial.println(" %/sec");

  Serial.print("TC Max Torque Up: ");
  Serial.print(TC_MAX_TORQUE_UP_PER_SEC, 1);
  Serial.println(" %/sec");


  Serial.println("------------------");
}
